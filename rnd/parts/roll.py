"""Arm-side shoulder: KX-V male ferrule → roll link (roll stage 18T:80T, or an inline GIM8108-36) → clevis → yaw stage
(20T:40T) → upper-arm flange.

Frame = the twin's pitch body for the LEFT arm (outboard +Y): pitch axis = Y through the origin, detach plane at y = 0.
Roll axis = X through (y = ROLL_Y, z = 0). Yaw axis = Z through (x = 0, y = ROLL_Y), upper-arm flange top at z = -YAW_DROP.

Roll drive (params.arm.roll_drive, D24): "belt" (loops 10–44, default) or "inline" (PIP_ROLL_DRIVE=inline): the GIM8108-36
sits on the roll axis, its housing bolted to the roll link, its output flange bolted to the clevis (see the INLINE section).
Both GIM6010 stages here (belt roll, yaw) are D24 bolted cartridges (belt_stage.bolted_stage, wall_a_cuts) since wave 2;
the shaft → link joints and their capacities vs the joint peaks are in drive_capacity().
"""
from __future__ import annotations

import math

from build123d import Align, Axis, Box, Cylinder, Part, Pos, Rot, fillet

from . import cartridge_m as cm
from .belt_stage import pulley
from .shoulder import FERRULE, ferrule

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
from ..params import BASELINE as _D
from .belt_stage import V3 as _V3, bolted_stage, wall_a_cuts
INLINE = _D.arm.roll_drive == "inline"
EAR_SITE = "yaw"            # D24 v2 flange-ear pattern (cartridge_m.EAR_SITES) of the yaw stage (and the belt-roll fallback), read by rnd/assembly_seq.py
ROLL_Y = _D.arm.roll_offset_y * 1000                 # loop 10: 50 → 75 (the Ø114 roll pulley); loop 40: 83 (the 80T pulley clears the ferrule by 2 mm)
# D24 inline roll (owner-approved 2026-11-27): GIM8108-36, 36:1 planetary, 24 V SDC303 (rnd/specs/actuators.yaml). Seller
# installation drawing (owner, 2026-11-27, docs/rnd/specs/gim8108-36_installation.png), ≈ 57 mm overall with the driver:
#   driver face: Ø75 × 12.5 driver section (6 × M2 through on Ø70, 3 × M2 tapped on Ø56, unused here)
#   body: Ø96 × 29, 8 × M4 tapped 3.5 deep on Ø84 on both faces (we use the driver-side face)
#   output face: stepped Ø60 / Ø42 boss 15.5 long (split 10 / 5.5 ESTIMATE), Ø30 output boss with 3 × M4 tapped 4 deep on
#   Ø28.5 + 3 × Ø4 H7 dowel holes. Which face is output vs driver is TO CONFIRM (low-res scan): assumed the Ø30-boss face.
# Output-bearing radial rating: not on the drawing (open seller question). Boss protrusion 1.0 ESTIMATE.
MOT36 = dict(od=96.0, drv_d=75.0, drv_h=12.5, body_h=29.0, step=((60.0, 10.0), (42.0, 5.5)), boss_d=30.0, boss_h=1.0,
             rear_n=8, rear_pcd=84.0, rear_tap=3.5, out_n=3, out_pcd=28.5, out_tap=4.0, dowel_n=3, dowel_d=4.0, dowel_l=4.0, out_brg=8.0)
REAR_ANG0 = 0.0                                                    # first of the 8 rear M4 (≥ 9 mm from the hard-stop lugs)
MOT36["h"] = MOT36["body_h"] + sum(h for _, h in MOT36["step"])   # body rear face → output step face (44.5); + the driver = 57
CUP_H = MOT36["drv_h"] + 0.5                                       # roll-link cup ring around the driver section
# M1 (Fable review 2026-11-27): the housing's M4 are tapped only 3.5 deep; wave 2's M4 × 18 under a 3.5 mm counterbore
# engaged 2.5 ± 0.55 mm (under 3 threads, or bottoming). Now: M4 × 18 ISO 4762 12.9 + ISO 7089 washer (0.8) + one DIN 988
# shim (0.3 nominal, chosen from 0 / 0.3 / 0.5 at assembly) under the head in a Ø9.6 × 5.1 counterbore (head flush): the
# printed grip is 13.9, the engagement 18 − 13.9 − 0.8 − 0.3 = 3.0 mm. Stack: screw length js15 ±0.35 (ISO 4759-1),
# printed grip ±0.15 (ESTIMATE), washer ±0.1 (ISO 7089) → ±0.6 worst case; so each screw's protrusion past the cup's
# motor face is measured with the washer + shim seated (depth gauge, motor off) and the shim picked to land 3.0 ± 0.15
# (+ gauge ± 0.05): 2.8–3.2 mm, never bottoming (3.5). Preload 0.5 kN (≈ 0.45 N·m, ≈ 10 MPa under the washer on the
# PETG). Rows: rnd/joints.py. Alternative (the review's): Loctited M4 studs bottomed then backed off ¼ turn + nuts.
REAR_SCREW = dict(size="M4 × 18 ISO 4762 12.9", L=18.0, L_tol=0.35, washer=0.8, washer_tol=0.1, shim=0.3, shims=(0.0, 0.3, 0.5),
                  cb_depth=4.0 + 0.8 + 0.3, cb_d=9.6, grip_tol=0.15, preload=500.0, engage=None, grip=None)
REAR_SCREW["grip"] = CUP_H + 6.0 - REAR_SCREW["cb_depth"] + REAR_SCREW["washer"] + REAR_SCREW["shim"]   # 15.0 under the head
REAR_SCREW["engage"] = REAR_SCREW["L"] - REAR_SCREW["grip"]       # 3.0 mm into the 3.5 mm tapped holes
# Metal output adapter (7075-T6, Ø76 × 7, ESTIMATE): 3 × M4 × 10 countersunk (ISO 10642; 3.0 mm into the 4 mm tapped holes)
# + 3 × Ø4 m6 hardened dowels in the H7 holes into the Ø30 boss, Ø30 pilot; 8 × M5 tapped on PCD 64 take the printed clevis
# arm (bolted from +X through a steel clamp ring: M5 × 16 low-head, 6 mm into the adapter).
ADAPT = dict(od=76.0, t=7.0, n=8, pcd=64.0, d=5.0, mat_density=2.81e-3)
R_MOT = MOT36["od"] / 2
FER_FACE = FERRULE["t"] + 10.0                        # ferrule spigot end (rnd/parts/shoulder.py ferrule): 18 mm off the detach plane
ROLL_MIN = -15                                        # = twin.JR["roll"][0] (rnd/swept.py checks the whole range)
if INLINE:
    ROLL_Y = FER_FACE + 2.0 + R_MOT                   # 70: the motor clears the spigot by 2 mm (the sweep is checked below)
# loop 40 (D13 migration of roll + yaw; yaw ±150° restored with a clock-spring cassette):
#  - roll belt centre 90 → 102: the D13 yaw stage is 11 mm taller, so the yaw cartridge (roll output) reaches 48 mm from the
#    roll axis; the static roll collar/can must sit ≥ 2 mm outside that over the whole roll sweep (rnd/spacing.py swept).
#    Roll belt → stock HTD 475-5M.
#  - roll 72T → 80T (ratio 4.0 → 4.44): the D13 stages put 0.16 kg more on the shoulder, and the whole-workspace roll
#    margin (rated / p95, 1 kg, yaw ±150°) fell to 1.49×; 80T restores ≈ 1.6× with no motor change.
ROLL_TEETH = 80
ROLL_C, YAW_C = 103.3, 80.0          # 80T:18T roll on a stock 475 mm belt
# D24 (2026-11-27): both stages are belt_stage.small_stage_v3 with the bolted cartridge (D24 v2, Fable C3: M4 ear bolts from
# the cartridge side into nuts on wall A's pulley face, site EAR_SITE; centring lip in the wall's counterbore, Oldham drive). Stage frame: z = 0 is the
# flange plane = the cartridge-side face of wall A; the bearings, pulley and belt plane sit where the KX74 V2 stack had them.
JA, JB = _V3["jA"], _V3["jB"]                          # D13 bearing spans (stage frame, below the flange plane: negative z)
PUL_C = -_V3["belt_c"]                                 # 16.5: small-pulley belt centre below the flange plane
ZC = cm.bolted_levels()["rear"]                        # flange plane → can rear (69.1; the KX74 cartridge stood 80.4 above wall A)
R_CAN = cm.MOTOR["od"] / 2 + cm.WALL + 0.3               # bolted can sleeve radius (43.3)


def _stage(T, teeth=20):
    """D24 bolted cartridge + Oldham + D13 small stage v3 (belt_stage.bolted_stage), stage frame mapped by T."""
    return bolted_stage(T, teeth, site=EAR_SITE)


def _fl(b, r, dims):
    r = min(r, min(dims) / 2 - 0.3)
    return fillet(b.edges().filter_by(Axis.Z), r) if r > 0.5 else b


def _plate_x(x0, x1, y0, y1, z0, z1, r=10):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Rot(0, 90, 0) * _fl(Box(z1 - z0, y1 - y0, x1 - x0), r, (z1 - z0, y1 - y0))
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Rot(0, 90, 0) * fillet(Box(z1 - z0, y1 - y0, x1 - x0).edges().filter_by(Axis.Z), r)


def _plate_z(x0, x1, y0, y1, z0, z1, r=10):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * _fl(Box(x1 - x0, y1 - y0, z1 - z0), r, (x1 - x0, y1 - y0))


# roll stage: cartridge behind (−X), flange plane at x = XF (= wall W1's −X face); 6 mm walls with the D13 bearings flush
# on the pulley side
XF = -PUL_C                                            # stage z → world x = XF − z; belt plane x = 0
W1 = (XF - JA[0] - 6.0, XF - JA[0])                    # inner wall = stage wall A (bearing A at x −10.5 … −15.5, flush at −10.5)
W2 = (XF - JB[1], XF - JB[1] + 6.0)                    # outer wall (bearing B at x 17.5 … 22.5)
ROLL_T = Pos(XF, ROLL_Y, ROLL_C) * Rot(0, -90, 0) * Rot(0, 0, 180)   # stage +Z → world −X, stage +X → world −Z (toward the 80T)
HEX_AF = 34.0                                          # shaft hex under the split pulleys (corners Ø39.3: passes the 61808 bore)
CAV_H = 11.3                                           # clock-spring cavity height: A14's two CAN pairs make the ribbon 9.8 mm wide (rnd/harness.py); was 9.5 for 7.8
ROLL_SPR = (22.0, 38.0)                                # roll clock spring r_in, r_out
YAW_SPR = (22.5, 42.5)                                 # yaw clock spring (inner wall = the 40T pulley's hub); r_out 40 → 42.5: 2.3 mm ribbon packs only 279° at r 40 (rnd/harness.py), ≈ 400° with a 0.75 m ribbon


def _hex(af, h):
    from build123d import RegularPolygon, extrude
    return extrude(RegularPolygon(af / math.sqrt(3), 6), h)


def roll_link() -> Part:
    if INLINE:
        return roll_link_inline()
    p = Pos(0, 8, 0) * Rot(-90, 0, 0) * Cylinder(45, 4, align=MIN)            # boss behind the ferrule (y 8..12)
    for x0, x1 in (W1, W2):
        w = _plate_x(x0, x1, 8, ROLL_Y + 40, -22, ROLL_C + WALL_TOP, 14)
        w -= Pos(0, ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(26, 100)                # 61808 seat, roll shaft
        w -= Pos((x0 + x1) / 2, ROLL_Y, -15) * Box(x1 - x0 + 1, 52, 30)            # D19: U-slot open downward, closed by roll_seat_cap
        for yy in (ROLL_Y - SEAT_BOLT_Y, ROLL_Y + SEAT_BOLT_Y):
            w -= Pos((x0 + x1) / 2, yy, -22) * Cylinder(2.8, 8, align=MIN)          # M4 heat-set inserts for the cap
        if (x0, x1) == W1:
            w -= ROLL_T * wall_a_cuts(None, EAR_SITE)                             # D24 v2: eccentric seat, spigot counterbore, ear bolts + nut pockets
        else:
            w -= Pos(0, ROLL_Y, ROLL_C) * Rot(0, 90, 0) * Cylinder(19.1, 100)     # eccentric-bushing seat, D13 bearing B
            w -= ROLL_T * flange_screw_access(W2[1] - XF + 1)                     # D24: driver holes to the 6 flange screws (swap with the belt on)
        p += w
    p += _plate_x(W1[0], W2[1], 8, 12, 46, ROLL_C + WALL_TOP, 4)                 # web joining the walls to the boss, above the roll pulley
    return p + roll_spring_bar()


WALL_TOP = 37.0                        # roll-link walls reach ROLL_C + 37: the bolted can flange (r 34) sits on wall W1


def flange_screw_access(depth, r=4.6):
    """Stage frame: Ø9.2 driver holes on the 6 cartridge-flange screw axes, from the flange plane to depth below it (the
    walls / plate opposite wall A), so the cartridge comes off with the belt and pulley in place (D24)."""
    f = cm.FLANGE_B
    c = None
    for a in cm.FLANGE_ANG:
        t = math.radians(a)
        h = Pos(f["bolt_r"] * math.cos(t), f["bolt_r"] * math.sin(t), -depth) * Cylinder(r, depth - 6.5, align=MIN)
        c = h if c is None else c + h
    return c


SEAT_BOLT_Y = 33.0                     # D19 cap bolts (2 × M4 per seat) either side of the roll axis, into the wall bottoms


def roll_seat_cap(W) -> Part:
    """D19 (belt roll): half-ring cap closing a wall's 61808 U-slot from below (2 × M4 up into heat-set inserts). The roll
    belt pulls the shaft up, into the wall; the hanging arm's weight goes into the cap."""
    x0, x1 = W
    c = Pos((x0 + x1) / 2, ROLL_Y, -27) * Box(x1 - x0, 2 * (SEAT_BOLT_Y + 6), 10) + Pos((x0 + x1) / 2, ROLL_Y, -11) * Box(x1 - x0, 51.6, 22)
    c -= Pos(0, ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(26, 100)
    for yy in (ROLL_Y - SEAT_BOLT_Y, ROLL_Y + SEAT_BOLT_Y):
        c -= Pos((x0 + x1) / 2, yy, -40) * Cylinder(2.2, 30, align=MIN)
    return c


def roll_shaft() -> Part:
    """D19 (belt roll, after Astra review #2b finding 1): Ø40 tube with the 80T pulley printed on it in one piece (no hex
    joint), plus the two inner-race shoulders (r 22.5) that replace loop 40's shims. Bearing A slides on from −X, B from +X;
    the sub-assembly (+ belt) rises into the walls' U-slots and roll_seat_cap closes them; the clevis comes up from below
    onto the shaft ends and roll_clevis_cap closes its arms. The spring hub slides on the +X end last."""
    x0, x1 = CLEVIS_X[0][0] - 0.5, ROLL_SPR_X[1] - 0.5
    s = Cylinder(20, x1 - x0, align=MIN)
    p = pulley(ROLL_TEETH, 16, flange=False)
    r_rim = ROLL_TEETH * 5.0 / math.pi / 2 - 0.57 - 2.06 - 5.0
    for z in (0.0, 10.5):                                                    # lightening: 5 mm web between a Ø56 hub and a 5 mm rim
        p -= Pos(0, 0, z) * (Cylinder(r_rim, 5.5, align=MIN) - Cylinder(28, 5.5, align=MIN))
    s += Pos(0, 0, -8 - x0) * p
    s += Pos(0, 0, W1[1] + 0.1 - x0) * Cylinder(22.5, -8 - W1[1] - 0.1, align=MIN)          # shoulder to bearing A's inner race
    s += Pos(0, 0, 8 - x0) * Cylinder(22.5, W2[0] - 0.1 - 8, align=MIN)                      # shoulder to bearing B's inner race
    s -= Pos(0, 0, CLEVIS_X[0][1] + 0.5 - x0) * Cylinder(14, CLEVIS_X[1][0] - CLEVIS_X[0][1] - 1.0, align=MIN)   # bore only between the clamps
    s -= Cylinder(5, x1 - x0, align=MIN)                                                   # Ø10 vent / print core
    return Pos(x0, ROLL_Y, 0) * Rot(0, 90, 0) * s


def roll_pulley() -> Part:
    """Loops 40–44: the split 80T pulley on a hex (could not be fitted: Astra #2b finding 1). Kept for reference only; D19
    prints it on the roll shaft (roll_shaft)."""
    p = pulley(ROLL_TEETH, 16, flange=False) - _hex(HEX_AF + 0.4, 16)
    r_rim = ROLL_TEETH * 5.0 / math.pi / 2 - 0.57 - 2.06 - 5.0
    for z in (0.0, 10.5):
        p -= Pos(0, 0, z) * (Cylinder(r_rim, 5.5, align=MIN) - Cylinder(28, 5.5, align=MIN))
    return Pos(-8, ROLL_Y, 0) * Rot(0, 90, 0) * p


# roll clock spring (A3): outboard of the +X clevis arm. The rotating hub (disc + sleeve) screws to the clevis arm; the
# static ring + cover hang on a bar that runs inboard (−Y, at z ≈ 0: a direction the roll output never sweeps) to the
# roll-link boss. The ribbon's static end leaves through the ring toward the detach plug; its rotating end runs down a
# groove in the clevis arm to the yaw cassette.
ROLL_SPR_X = (W2[1] + 12 + 1.5, W2[1] + 12 + 1.5 + CAV_H)   # cavity x-span (37.0 … 46.5)


def roll_spring_hub() -> Part:
    x0, x1 = ROLL_SPR_X
    h = Pos(x0 - 1.5, ROLL_Y, 0) * Rot(0, 90, 0) * (Cylinder(ROLL_SPR[1] - 0.8, 1.5, align=MIN) - Cylinder(20.1, 1.5, align=MIN))
    h += Pos(x0, ROLL_Y, 0) * Rot(0, 90, 0) * (Cylinder(ROLL_SPR[0], x1 - x0 - 0.5, align=MIN) - Cylinder(20.1, x1 - x0, align=MIN))
    return h


def roll_spring_bar() -> Part:
    """Bar from the roll-link boss to the spring ring (D19: the ring is its own part, screwed on last)."""
    x0, x1 = ROLL_SPR_X
    r1 = ROLL_SPR[1]
    return Pos((x0 + 1 + x1 + 1.6) / 2, (12 + ROLL_Y - r1) / 2, 0) * Box(x1 + 1.6 - x0 - 1, ROLL_Y - r1 - 12 + 1, 14)


def roll_spring_static() -> Part:
    x0, x1 = ROLL_SPR_X
    r0, r1 = ROLL_SPR
    ring = Pos(x0 + 0.5, ROLL_Y, 0) * Rot(0, 90, 0) * (Cylinder(r1 + 2, x1 - x0 + 1.6 - 0.5, align=MIN) - Cylinder(r1, x1 - x0 - 0.5, align=MIN))
    ring -= Pos(x1 - 1, ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(r0 + 0.5, 5, align=MIN)      # cover clears the rotating sleeve
    ring -= Pos((x0 + x1) / 2, ROLL_Y, r1 + 1) * Box(CAV_H - 1, 10, 4)                      # ribbon exit (top)
    bar = Pos((x0 + 1 + x1 + 1.6) / 2, (12 + ROLL_Y - r1) / 2, 0) * Box(x1 + 1.6 - x0 - 1, ROLL_Y - r1 - 12 + 1, 14)
    return ring + bar


# yaw stage: yaw axis Z at (0, ROLL_Y); belt plane horizontal; cartridge above its D13 stage at x = -YAW_C.
# Loop 40: top plate 3 mm under the roll pulley; 6 mm plates with the D13 bearings flush on the pulley side. The yaw clock
# spring sits between the 40T pulley and the bottom plate (cavity 9.5 mm), so the drop added is the D13 stage's length
# and the 80T roll pulley: yaw_drop 100 → 113.0 mm.
_R_ROLL = ROLL_TEETH * 5.0 / math.pi / 2 - 0.5715       # roll pulley outer radius
_R_TOP = R_MOT if INLINE else _R_ROLL                  # inline: the top plate clears the static motor housing (r 50)
_C_TOP = 4.0 if INLINE else 3.0                        # inline: the yaw shaft's top end (2 mm above the plate) sweeps 2 mm past the housing
P1 = (-(_R_TOP + _C_TOP) - 6.0, -(_R_TOP + _C_TOP))    # top plate z-span (bearing A flush on its underside)
ZF = P1[0] - JA[0]                     # flange plane of the yaw cartridge = the top plate's top face (D24; the KX face was 23.7 higher)
BAND_CH = 12.0                         # inline: 45° chamfer on the yaw band's inboard top edge (yaw_band)


def ferrule_sweep_gap(roll_y, roll_deg=ROLL_MIN):
    """Inline roll: smallest gap (mm) between the roll output's inboard edge (yaw plates/walls at ±46, the band at ±50 with
    its BAND_CH chamfer) and the arm-side ferrule (flat Ø91 × 3, 20° taper to y 8, Ø64 spigot to y 18) at full adduction,
    in the plane x = 0 where the ferrule is widest. The roll link's own boss/frame stay above z −28 (roll_link_inline)."""
    import numpy as np
    top_b, z_bot = P1[1] + 1.5, P1[1] - 60.0
    moving = [(-46.0, z) for z in np.linspace(z_bot, P1[1], 40)] + [(y, P1[1]) for y in np.linspace(-46, 0, 30)]
    moving += [(-50.0, z) for z in np.linspace(z_bot, top_b - BAND_CH, 40)]
    moving += [(-50.0 + BAND_CH * t, top_b - BAND_CH * (1 - t)) for t in np.linspace(0, 1, 30)] + [(y, top_b) for y in np.linspace(-50 + BAND_CH, 0, 30)]
    r8 = FERRULE["od"] / 2 - (FERRULE["t"] - 3.0) * math.tan(math.radians(90 - FERRULE["taper_deg"])) * 0.35
    prof = [(0.0, 0.0), (0.0, FERRULE["od"] / 2), (3.0, FERRULE["od"] / 2), (8.0, r8), (8.0, 32.0), (FER_FACE, 32.0), (FER_FACE, 0.0)]
    seg = lambda p, a, b: float(np.hypot(*(p - (a + np.clip(np.dot(p - a, b - a) / max(np.dot(b - a, b - a), 1e-9), 0, 1) * (b - a)))))
    t = math.radians(roll_deg)
    gap = 1e9
    for yr, z in moving:
        Y, Z = roll_y + yr * math.cos(t) - z * math.sin(t), yr * math.sin(t) + z * math.cos(t)
        p = np.array([Y, abs(Z)])
        d = min(seg(p, np.array(a), np.array(b)) for a, b in zip(prof[:-1], prof[1:]))
        inside = 0 <= Y <= FER_FACE and abs(Z) <= (FERRULE["od"] / 2 if Y <= 3 else r8 + (FERRULE["od"] / 2 - r8) * (8 - Y) / 5 if Y <= 8 else 32.0)
        gap = min(gap, -d if inside else d)
        if 8 <= Y <= FER_FACE + 2 and Z > -28 - 2:                                  # roll link boss / frame (z ≥ −28)
            gap = min(gap, -28 - Z)
    return gap


if INLINE:                             # ROLL_Y derived: the motor clearance (above), raised in 0.5 mm steps until the sweep clears
    while ferrule_sweep_gap(ROLL_Y) < 2.0:
        ROLL_Y += 0.5
YAW_T = Pos(-YAW_C, ROLL_Y, ZF)
# Yaw stack below the 40T (2026-11-27, R16 belt across-body): 40T underside → RING_GAP → spring ring top → cavity CAV_H
# (rnd/harness.py: 9.8 mm ribbon) → bottom-plate top (= cavity floor, yaw bearing B flush on it) → plate 6 mm, but only
# 6 − FL_POCKET inside a Ø94 pocket in its underside, in which the upper-arm flange sits (2.3 mm under the pocket ceiling,
# 2 mm radial) → flange 5 mm. Wave 1 had dropped the whole plate 2.6 mm for the 11.3 mm cavity (yaw_drop 113.4 → 116.0);
# the pocket and the 0.8 → 0.4 ring gap take that back. The 61808 keeps a 5.4 mm seat (pocket ceiling + a Ø60 boss under
# it), the flange top has a Ø64 × FL_REC recess around the boss and the bearing, both 2.1 mm clear (R15 moving ≥ 2).
RING_GAP = 0.4                          # 40T underside → static spring ring top (running, R15 ≥ 0.3); was 0.8
FL_POCKET = 2.2                         # flange pocket in the bottom plate's underside (plate 6 → 3.8 mm inside r 47)
FL_GAP, FL_T, FL_R = 2.3, 5.0, 45.0     # flange: gap under the pocket ceiling, thickness, radius
BOSS = (26.0, 30.0, 1.6)                # yaw bearing B boss under the pocket ceiling: r_in (61808 OD 52), r_out, height
RING_TOP = ZF - PUL_C - 8.0 - RING_GAP  # spring ring top (the 40T's lower face is at the belt plane − 8)
_CAV0 = RING_TOP - (ZF + JB[1])         # yaw cavity height with the bottom plate at the D13 bearing B (9.1 mm)
DP2 = max(0.0, CAV_H - _CAV0)          # the bottom plate drops this much for CAV_H; a boss under D13 bearing B keeps its seat
P2 = (ZF + JB[1] - DP2 - 6.0, ZF + JB[1] - DP2)    # bottom plate; yaw bearing B flush on its top (= the cavity floor)
YAW_BRG_B = (P2[1] - 7.0, P2[1])       # 61808 span (rnd/assembly_seq.py)
FL_TOP = P2[0] + FL_POCKET - FL_GAP    # flange top face (outer part)
FL_REC = round(max(0.0, FL_TOP - YAW_BRG_B[0]) + 0.5, 2)   # flange-top recess: the bearing's outer race clears it by 0.5 (running)
YAW_BELT = ZF - PUL_C                  # −82.6
YAW_DROP = round(-(FL_TOP - FL_T), 1)  # = params.arm.yaw_drop (belt 116.0 in wave 1 → 113.4)


YAW_WALL_Y = 44.0                      # side-wall centre offset from the yaw axis (plates span ±46)
X_PL1 = 48.0                           # +X end of the yaw plates (belt)
# names of the parts on the roll output (rnd/swept.py: they rotate with roll; everything else in module() is static)
ROLL_OUT = {"roll_shaft", "roll_pulley72", "roll_spring_hub", "yaw_block", "yaw_shaft", "yaw_flange", "yaw_spring_ring", "yaw_plate_bot",
            "yaw_band", "yaw_stop_pin", "roll_arm_rear", "roll_outflange", "roll_clevis_cap_in", "roll_clevis_cap_out"}


# ---------------------------------------------------------------------------------------------------------------------
# D24 INLINE ROLL (PIP_ROLL_DRIVE=inline). Along the roll axis (x), from −X:
#   yaw cartridge (roll output, x ≤ its CAD max) | roll_arm_rear on a 61806 (30×42×7) on the static stub, with the yaw torque
#   fork | 2.5 mm | roll-link floor (6 mm, 6 × M4 into the motor's rear face) | GIM8108-36 housing (static) | output flange
#   | +X clevis arm (fused to the yaw block, 6 × M4 into the flange) | roll clock-spring cassette (hub on the arm, ring on
#   the roll link's bar, as in the belt stage).
# Two-point roll support: the motor's output bearing (+X) and the 61806 (−X), 68 mm apart (belt: 2 × 61808, 39 mm apart);
# the motor's output bearing then carries ≈ 19 N·m / 0.068 ≈ 280 N radial in the 1 kg workspace (rating not published:
# ESTIMATE, confirm with the seller). Assembly: motor onto the floor (screws from −X) → 61806 into roll_arm_rear, arm onto
# the stub from −X, end washer + M5 → yaw block (+ yaw cartridge) up from below: its arm slides down the flange face,
# 6 × M4 into the flange from +X, roll_arm_rear's foot 2 × M5 into the top plate (the torque pin rises into the fork
# slot) → spring hub on the arm, ribbon, spring ring onto the bar. No pulley, belt or D13 stage, so R14 has nothing to
# block (rnd/assembly_seq.py checks the 61806 slide-on).
# Roll travel: software −15…100° (twin.JR); hard stops at ≈ −19° / 104°. M1 (Fable review 2026-11-27): the stops were two
# printed Ø8 lugs (≈ 800 N at the 35 N·m drive limit, ≈ 16 MPa vs the 12 MPa allowable; 2.3 kN at stall). Now two
# hardened ISO 8734 Ø10 × 28 dowels at r 43, pressed 19.5 mm into the floor + cup ring and standing 8 mm proud on its
# −X face, meet 20 × 8 × 1.5 stainless wear strips bonded flush into roll_arm_rear's edges (pin on steel; the strip spreads
# the load on the print). Rows: rnd/joints.py.
# ---------------------------------------------------------------------------------------------------------------------
BRG_R = dict(d=30.0, D=42.0, B=7.0)    # 61806, rear roll support
# +X clevis arm ↔ metal output adapter (Astra #2b finding 2; the motor's own 3 × M4 on PCD 28.5 are too small to go
# straight into a print): 8 × M5 low-head (DIN 7984) 10.9 from +X through the arm into the adapter's threads, heads on a
# Ø80/Ø52 × 2 steel clamp ring in a counterbore (≈ 5 MPa on the PETG at the design preload); fitted Ø5.1 holes give a
# bearing path behind the friction. Capacities: drive_capacity().
OUT_CLAMP = dict(arm_t=14.0, ring=(26.0, 40.0, 2.0), head_h=3.5, preload=1500.0, creep=0.75, mu=0.25)
R_ARM = 42.0                           # clevis-arm boss radius about the roll axis (covers the Ø76 adapter and the ring)
R_ARM_REAR = 36.0                      # roll_arm_rear boss radius (61806 seat): inside the hard-stop pins (r 43 − 5)
STOP_R, STOP_LUG_R = 43.0, 5.0          # M1: steel Ø10 stop pins (were printed Ø8 lugs at r 44)
STOP_PIN = dict(d=10.0, seat=6.0 + CUP_H - 0.5, proud=8.0, strip=(20.0, 8.0, 1.5))
STOP_AT = (ROLL_MIN - 4, 100 + 4)      # hard-stop contact angles (deg)
if INLINE:
    _ycx = max(p.bounding_box().max.X for _, p, _c in _stage(YAW_T, 20) if p.bounding_box().max.Z > P1[1])
    X_ARM_R = (math.ceil((_ycx + 1.2) * 2) / 2, math.ceil((_ycx + 1.2) * 2) / 2 + 8.0)   # 1.2 mm past the yaw cartridge (its can, D24)
    X_FLOOR = (X_ARM_R[1] + 2.5, X_ARM_R[1] + 2.5 + 6.0)
    X_CUP = (X_FLOOR[1], X_FLOOR[1] + CUP_H)                        # ring around the motor's driver section
    X_MOT = (X_CUP[1], X_CUP[1] + MOT36["h"])                       # Ø96 body rear face → output step face
    X_ADAPT = (X_MOT[1] + MOT36["boss_h"], X_MOT[1] + MOT36["boss_h"] + ADAPT["t"])   # on the Ø30 boss face; clears the step by boss_h
    X_ARM_F = (X_ADAPT[1], X_ADAPT[1] + OUT_CLAMP["arm_t"])
    X_PL1 = X_ARM_F[1]
    ROLL_SPR_X = (X_ARM_F[1] + 1.5, X_ARM_F[1] + 1.5 + CAV_H)
    W1 = (X_ARM_R[0] + 0.5, X_ARM_R[0] + 0.5 + BRG_R["B"])          # 61806 (rnd/stiffness.py reads the bearing spans)
    W2 = (X_MOT[1] - MOT36["out_brg"], X_MOT[1])                    # motor output bearing, in the stepped boss (ESTIMATE position)
    ROLL_C = 0.0                                                    # roll actuator axis above the roll axis (covers.py)
    XF, ROLL_T = None, None


def _ang(deg, r):
    t = math.radians(deg)                                           # roll angle convention: 0 = −Z, + toward +Y
    return ROLL_Y + r * math.sin(t), -r * math.cos(t)


def roll_link_inline() -> Part:
    """Static roll link: ferrule boss (Ø90 × 4 on the ferrule's back, cut below z −28), a 10 mm frame around the spigot
    (y 8…18, z −28…40) running from the floor to the spring bar, the floor (Ø100 × 6, 6 × M4 counterbored on PCD 84), the
    Ø30 support stub with its Ø34 shoulder, two hard-stop lugs and the spring bar."""
    xf0, xf1 = X_FLOOR
    zlo, zhi = -28.0, 40.0
    p = Pos(0, 8, 0) * Rot(-90, 0, 0) * Cylinder(45, 4, align=MIN)
    p &= Pos(0, 10, (zlo + 60) / 2) * Box(100, 10, 60 - zlo)
    sx1 = ROLL_SPR_X[1] + 1.6
    p += _plate_x(xf0, sx1, 8, 18, zlo, zhi, 6) - Pos(0, 13, 0) * Rot(-90, 0, 0) * Cylinder(32.3, 12)            # frame around the spigot
    p += Pos(xf0, ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(R_MOT, xf1 - xf0, align=MIN)                            # floor
    p += Pos(xf1, ROLL_Y, 0) * Rot(0, 90, 0) * (Cylinder(R_MOT, CUP_H, align=MIN) - Cylinder(MOT36["drv_d"] / 2 + 0.5, CUP_H, align=MIN))   # cup around the driver
    p += _plate_x(xf0, xf1, 8, ROLL_Y, zlo, zhi, 4)                                                             # floor ↔ frame web
    for k in range(MOT36["rear_n"]):
        a = 2 * math.pi * k / MOT36["rear_n"] + REAR_ANG0
        y, z = ROLL_Y + MOT36["rear_pcd"] / 2 * math.cos(a), MOT36["rear_pcd"] / 2 * math.sin(a)
        p -= Pos(0, y, z) * Rot(0, 90, 0) * Cylinder(2.25, 400)
        p -= Pos(xf0, y, z) * Rot(0, 90, 0) * Cylinder(REAR_SCREW["cb_d"] / 2, REAR_SCREW["cb_depth"], align=MIN)   # M1: head + washer + shim; M4 × 18 engages 3.0 of 3.5
    p -= Pos(xf1 - 2.5, (ROLL_Y + 8) / 2, 0) * Box(5, ROLL_Y - 8, 10)                                           # motor-lead groove to the frame
    x_st = W1[0]
    p += Pos(x_st, ROLL_Y, 0) * Rot(0, 90, 0) * (Cylinder(BRG_R["d"] / 2, xf0 - x_st, align=MIN) - Cylinder(9, xf0 - x_st, align=MIN))
    p += Pos(W1[1] + 0.3, ROLL_Y, 0) * Rot(0, 90, 0) * (Cylinder(17, xf0 - W1[1] - 0.3, align=MIN) - Cylinder(9, 20, align=MIN))   # inner-race shoulder
    for deg in STOP_LUG_ANG:
        y, z = _ang(deg, STOP_R)
        p += Pos(X_ARM_R[0] + 2.5, y, z) * Rot(0, 90, 0) * Cylinder(STOP_LUG_R, xf0 - X_ARM_R[0] - 2.5, align=MIN)
    x0, x1 = ROLL_SPR_X
    r1 = ROLL_SPR[1]
    p += Pos((x0 + 1 + x1 + 1.6) / 2, (12 + ROLL_Y - r1) / 2, 0) * Box(x1 + 1.6 - x0 - 1, ROLL_Y - r1 - 12 + 1, 14)   # spring bar
    return p


def _stop_lug_angles():
    """Lug centre angles: roll_arm_rear's straight edges (±26 from its centre line) touch a lug of radius STOP_LUG_R at
    radius STOP_R when sin(Δ) = (26 + r_lug) / r."""
    d = math.degrees(math.asin((26.0 + STOP_LUG_R) / STOP_R))
    return (STOP_AT[0] - d, STOP_AT[1] + d)


STOP_LUG_ANG = _stop_lug_angles()


def roll_arm_rear() -> Part:
    """−X clevis arm (separate print): R 34 boss with the 61806 seat (outer-race lip on the +X side), a ±26 plate down to a
    foot on the top yaw plate (2 × M5). (D24: the yaw cartridge's torque fork is gone; its flange screws carry the torque.)"""
    x0, x1 = X_ARM_R
    a = _plate_x(x0, x1, ROLL_Y - 26, ROLL_Y + 26, P1[1], 0, 10) + Pos(x0, ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(R_ARM_REAR, x1 - x0, align=MIN)
    a += _plate_x(x0 - 10, x1, ROLL_Y - 26, ROLL_Y + 26, P1[1], P1[1] + 5, 4)                                    # foot
    a -= Pos(x0, ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(BRG_R["D"] / 2, W1[1] - x0, align=MIN)                     # 61806 seat
    a -= Pos(0, ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(BRG_R["D"] / 2 - 2.0, 400)                                 # lip bore
    for yy in (ROLL_Y - 16, ROLL_Y + 16):
        a -= Pos(x0 - 5, yy, P1[1]) * Cylinder(2.75, 12, align=MIN)
    return a


def roll_motor_housing() -> Part:
    """GIM8108-36 housing (static, seller drawing): Ø75 × 12.5 driver section in the roll link's cup, Ø96 × 29 body bolted
    to the cup (8 × M4 on Ø84), stepped Ø60 / Ø42 boss on the output side."""
    m = MOT36
    h = Pos(X_MOT[0] - m["drv_h"], ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(m["drv_d"] / 2, m["drv_h"], align=MIN)
    h += Pos(X_MOT[0], ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(R_MOT, m["body_h"], align=MIN)
    x = X_MOT[0] + m["body_h"]
    for d, l in m["step"]:
        h += Pos(x, ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(d / 2, l, align=MIN)
        x += l
    return h


def roll_outflange() -> Part:
    """Rotating output: the motor's Ø30 boss (boss_h past the step) + the metal adapter (Ø76 × 7, 7075, ESTIMATE):
    3 × M4 countersunk + 3 × Ø4 dowels on PCD 28.5 into the boss, 8 × M5 tapped on PCD 64 for the clevis arm, Ø16 centre."""
    m, a = MOT36, ADAPT
    boss = Pos(X_MOT[1] + 0.3, ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(m["boss_d"] / 2, m["boss_h"] - 0.3, align=MIN)   # runs 0.3 off the static step
    ad = Pos(X_ADAPT[0], ROLL_Y, 0) * Rot(0, 90, 0) * (Cylinder(a["od"] / 2, a["t"], align=MIN) - Cylinder(8.0, a["t"], align=MIN))
    for k in range(m["out_n"] + m["dowel_n"]):
        t = 2 * math.pi * k / (m["out_n"] + m["dowel_n"])
        ad -= Pos(0, ROLL_Y + m["out_pcd"] / 2 * math.cos(t), m["out_pcd"] / 2 * math.sin(t)) * Rot(0, 90, 0) * Cylinder(2.2 if k % 2 == 0 else 2.0, 400)
    for k in range(a["n"]):
        t = 2 * math.pi * k / a["n"] + math.pi / a["n"]
        ad -= Pos(0, ROLL_Y + a["pcd"] / 2 * math.cos(t), a["pcd"] / 2 * math.sin(t)) * Rot(0, 90, 0) * Cylinder(a["d"] / 2 - 0.4, 400)
    return boss + ad


def roll_spring_ring() -> Part:
    """Static outer wall + cover of the roll clock spring, screwed to the roll link's bar (inline: separate, it goes on last)."""
    x0, x1 = ROLL_SPR_X
    r0, r1 = ROLL_SPR
    ring = Pos(x0 + 0.5, ROLL_Y, 0) * Rot(0, 90, 0) * (Cylinder(r1 + 2, x1 - x0 + 1.6 - 0.5, align=MIN) - Cylinder(r1, x1 - x0 - 0.5, align=MIN))
    ring -= Pos(x1 - 1, ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(r0 + 0.5, 5, align=MIN)
    ring -= Pos((x0 + x1) / 2, ROLL_Y, r1 + 1) * Box(CAV_H - 1, 10, 4)                      # ribbon exit (top)
    return ring - Pos((x0 + 1 + x1 + 1.6) / 2, (12 + ROLL_Y - r1) / 2, 0) * Box(x1 + 1.6 - x0 - 1 + 0.01, ROLL_Y - r1 - 12 + 1 + 0.01, 14.01)


def module_inline():
    out = [("ferrule_m", Rot(-90, 0, 0) * ferrule(male=True), "orange"),
           ("roll_link", roll_link_inline(), "white"), ("roll_gim_housing", roll_motor_housing(), "dark"),
           ("roll_outflange", roll_outflange(), "steel"), ("roll_arm_rear", roll_arm_rear(), "white"),
           ("roll_spring_hub", roll_spring_hub(), "dark"), ("roll_spring_ring", roll_spring_ring(), "dark"),
           ("yaw_block", clevis_and_yaw_block(), "white"), ("yaw_shaft", yaw_shaft(), "orange"), ("yaw_flange", yaw_flange() + yaw_tie_ring(), "steel"),
           ("yaw_plate_bot", yaw_plate_bot(), "white"), ("yaw_spring_ring", yaw_spring_ring(), "dark"), ("yaw_band", yaw_band(), "white")]
    out += [(f"yaw_{n}", p, c) for n, p, c in _stage(YAW_T, 20)]
    out.append(("yaw_stop_pin", yaw_stop_pin(), "steel"))
    return out


def printed_parts():
    """Printed parts of this stage for rnd/assembly_check.py (bed fit), for the active roll drive."""
    if INLINE:
        return {"roll link": roll_link_inline(), "roll arm rear": roll_arm_rear(), "clevis + yaw block": clevis_and_yaw_block(),
                "yaw shaft": yaw_shaft(), "yaw plate bottom": yaw_plate_bot()}
    return {"roll link": roll_link(), "roll shaft + 80T (one piece)": roll_shaft(), "roll seat cap": roll_seat_cap(W1),
            "clevis cap": roll_clevis_cap(0), "clevis + yaw block": clevis_and_yaw_block(), "yaw shaft": yaw_shaft()}


def dome(mod=None):
    """Shoulder dome = the static, arm-side roll body (ferrule, roll link, roll actuator, cap, spring ring): extents in mm,
    fore-aft (x) × height (z) × outboard of the detach plane (y), from the CAD bounding box."""
    mod = mod or module()
    st = [p for n, p, _ in mod if n not in ROLL_OUT and not n.startswith("yaw_")]
    bb = [p.bounding_box() for p in st]
    x0, x1 = min(b.min.X for b in bb), max(b.max.X for b in bb)
    z0, z1 = min(b.min.Z for b in bb), max(b.max.Z for b in bb)
    y1 = max(b.max.Y for b in bb)
    return dict(width_x=round(x1 - x0, 1), height_z=round(z1 - z0, 1), out_y=round(y1, 1), z_top=round(z1, 1), z_bot=round(z0, 1))


def _check_params():
    import warnings
    if abs(_D.arm.roll_offset_y * 1000 - ROLL_Y) > 0.25 or abs(_D.arm.yaw_drop * 1000 - YAW_DROP) > 0.25:
        warnings.warn(f"params.arm roll_offset_y / yaw_drop ({_D.arm.roll_offset_y * 1000:.1f} / {_D.arm.yaw_drop * 1000:.1f}) "
                      f"≠ the {_D.arm.roll_drive} roll CAD ({ROLL_Y:.1f} / {YAW_DROP:.1f} mm): update params.ROLL_DRIVES")


_check_params()


def yaw_plate_bot() -> Part:
    """D19 (loop 44): the bottom yaw plate is its own print, bolted on last with the 4 post bolts (M5) and 4 × M4 into the
    side walls. Assembly: one-piece yaw shaft (40T + spring hub) with bearing A on top goes up into the top plate from
    below; the spring ring and ribbon go on; this plate comes up over bearing B; the flange bolts to the tube end last.
    (Loops 10–43 fused it to the top half, so nothing could be assembled; Astra review #2b finding 14.)"""
    pl = _plate_z(-YAW_C - 48, X_PL1, ROLL_Y - 46, ROLL_Y + 46, P2[0], P2[1], 16)
    pl -= Pos(0, ROLL_Y, P2[0] - 1) * Cylinder(FL_R + 2.0, FL_POCKET + 1, align=MIN)          # flange pocket (R16: −2.2 mm yaw drop)
    pl += Pos(0, ROLL_Y, P2[0] + FL_POCKET - BOSS[2]) * Cylinder(BOSS[1], BOSS[2], align=MIN)   # bearing-B boss: 3.8 + 1.6 = 5.4 mm seat
    pl -= Pos(0, ROLL_Y, 0) * Cylinder(BOSS[0], 400)
    if DP2 > 0:                                                                   # boss under the D13 stage's bearing B (CAV_H)
        pl += Pos(-YAW_C, ROLL_Y, P2[1]) * Cylinder(23.0, DP2, align=MIN)
    pl -= Pos(-YAW_C, ROLL_Y, 0) * Cylinder(19.1, 400)
    pl -= YAW_T * flange_screw_access(ZF - P2[0] + 1)                             # D24: driver holes to the yaw cartridge's flange screws
    for xx, yy in ((-YAW_C / 2, ROLL_Y - 38), (-YAW_C / 2, ROLL_Y + 38), (-YAW_C - 24, ROLL_Y - 38), (-YAW_C - 24, ROLL_Y + 38)):
        pl -= Pos(xx, yy, P2[0]) * Cylinder(2.75, 20, align=MIN)                  # post bolts
    t = math.radians(STOP_PIN_AZ)
    pl -= Pos(STOP_PIN_R * math.cos(t), ROLL_Y + STOP_PIN_R * math.sin(t), P2[0]) * Cylinder(2.95, 20, align=MIN)   # stop pin seat (through; pin flush with the cavity floor)
    return pl


def clevis_and_yaw_block() -> Part:
    p = None
    for z0, z1 in (P1,):                                                           # D19: P2 is yaw_plate_bot()
        pl = _plate_z(-YAW_C - 48, X_PL1, ROLL_Y - 46, ROLL_Y + 46, z0, z1, 16)
        pl -= Pos(0, ROLL_Y, 0) * Cylinder(26, 400)                                # 61808 seat, yaw shaft
        pl -= YAW_T * wall_a_cuts(None, EAR_SITE)                                 # D24 v2: eccentric seat, spigot counterbore, 2 ear bolts + nut pockets
        p = pl if p is None else p + pl
    # confidence campaign (FEA): the bottom plate was a free solid. Four Ø14 posts outside the belt loop tie it to the top plate
    # (M5 through-bolts; the belt and both pulleys go in before the bottom plate)
    for xx, yy in ((-YAW_C / 2, ROLL_Y - 38), (-YAW_C / 2, ROLL_Y + 38), (-YAW_C - 24, ROLL_Y - 38), (-YAW_C - 24, ROLL_Y + 38)):
        p += Pos(xx, yy, P2[1]) * (Cylinder(7, P1[0] - P2[1], align=MIN) - Cylinder(2.75, P1[0] - P2[1], align=MIN))
    # loop 40 (A8, rnd/stiffness.py): two plates on four posts tilted the yaw seats ≈ 22 mrad (P2) under the arm moment
    # (≈ 17 mm at the hand). 4 mm side walls along both long edges close the block into a box section (belt clear by 9 mm;
    # the +X end stays open for the ribbon). The walls belong to the top half; the bottom plate still bolts on last.
    for yy in (ROLL_Y + YAW_WALL_Y, ROLL_Y - YAW_WALL_Y):
        p += Pos((-112 + 32) / 2, yy, (P2[1] + P1[0]) / 2) * Box(144, 4, P1[0] - P2[1])
    p -= Pos(0, ROLL_Y, P2[1]) * Cylinder(YAW_SPR[1] + 1.6, RING_TOP + 1.0 - P2[1], align=MIN)   # the Ø88 yaw spring ring sits in the walls (2 mm left at x 0)
    if INLINE:
        # D24 inline: only the +X clevis arm is fused to the block; it bolts to the motor's output flange (6 × M4). The −X
        # arm (roll_arm_rear, with the yaw torque fork) is a separate print on the 61806 support bearing.
        x0, x1 = X_ARM_F
        arm = _plate_x(x0, x1, ROLL_Y - 26, ROLL_Y + 26, P1[0], 0, 10) + Pos(x0, ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(R_ARM, x1 - x0, align=MIN)
        arm -= Pos(0, ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(8.0, 400)                 # ribbon / cable pass to the spring cassette
        for k in range(ADAPT["n"]):
            a = 2 * math.pi * k / ADAPT["n"] + math.pi / ADAPT["n"]
            arm -= Pos(0, ROLL_Y + ADAPT["pcd"] / 2 * math.cos(a), ADAPT["pcd"] / 2 * math.sin(a)) * Rot(0, 90, 0) * Cylinder(ADAPT["d"] / 2 + 0.05, 400)   # fitted Ø5.1 (bearing path)
        ri, ro, t = OUT_CLAMP["ring"]                                                 # steel clamp ring under the 6 low heads
        arm -= Pos(x1 - t - OUT_CLAMP["head_h"], ROLL_Y, 0) * Rot(0, 90, 0) * (Cylinder(ro + 0.2, 20, align=MIN) - Cylinder(ri - 0.2, 20, align=MIN))
        p += arm
        return p
    for x0, x1 in CLEVIS_X:                                                       # clevis arms outside the walls
        arm = _plate_x(x0, x1, ROLL_Y - 26, ROLL_Y + 26, P1[0], 0, 10)            # arms end on the top plate (yaw pulley below is clear)
        arm += Pos(x0, ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(CLEVIS_R, x1 - x0, align=MIN)
        arm += Pos((x0 + x1) / 2, ROLL_Y, -PINCH["ear_h"] / 2) * Box(x1 - x0, 2 * CLEVIS_EAR, PINCH["ear_h"])   # D19: ears at the split line (z 0)
        arm -= Pos((x0 + x1) / 2, ROLL_Y, 20) * Box(x1 - x0 + 1, 80, 40)          # D19: U-slot open upward; roll_clevis_cap closes it
        arm -= Pos(0, ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(20.2, 100)           # lower half-bore on the roll shaft end
        for yy in (ROLL_Y - CLEVIS_EAR + 6, ROLL_Y + CLEVIS_EAR - 6):
            arm -= Pos((x0 + x1) / 2, yy, -PINCH["insert_l"]) * Cylinder(PINCH["insert_d"] / 2, PINCH["insert_l"], align=MIN)   # M5 heat-set inserts for the cap's pinch bolts
        arm -= Pos(0, ROLL_Y, 0) * Cylinder(22.0, 400)                            # loop 31/36: 2 mm clear of the rotating Ø40 yaw shaft (R15)
        p += arm
    return p


# D19 (belt roll): the clevis arms' roll-shaft bores are split at z = 0; the clevis comes up from below onto the shaft ends
# (already in the walls) and a cap per arm closes the bore. The caps' two M4 pinch bolts clamp the shaft (0.3 mm gap at the
# split line: this is also the clamp the clevis lacked, Astra #2b finding 2). The yaw torque fork moves onto the −X cap.
# Astra #2b finding 2 (2026-11-27): the clamp is the only torque path roll shaft → clevis, so it is sized for the roll
# actuator's joint peak (drive_capacity): M5 pinch bolts (were M4), the −X arm 10 → 14 mm (room left by the D24 bolted yaw
# cartridge: no torque fork, no KX collar), and the shaft is solid under both clamps (a Ø28 bore crushed at the clamp pressure).
PINCH = dict(n_per_arm=2, d=5.0, insert_d=6.4, insert_l=10.0, ear_h=12.0, preload=1500.0, creep=0.75, mu=0.30)
CLEVIS_X = ((W1[0] - 16, W1[0] - 2), (W2[1] + 2, W2[1] + 12))
CLEVIS_R, CLEVIS_EAR = 30.0, 34.0      # arm boss radius about the roll axis; ears ±34 at the split line


def roll_clevis_cap(i) -> Part:
    """Upper half of clevis arm i (0 = −X, with the yaw torque fork; 1 = +X): half-boss r 30 + ears, bore r 20.0 (clamps
    the Ø40 shaft when the pinch bolts close the 0.3 mm split gap)."""
    x0, x1 = CLEVIS_X[i]
    c = Pos(x0, ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(CLEVIS_R, x1 - x0, align=MIN) + Pos((x0 + x1) / 2, ROLL_Y, 3) * Box(x1 - x0, 2 * CLEVIS_EAR, 6)
    c &= Pos((x0 + x1) / 2, ROLL_Y, 20.3) * Box(x1 - x0, 80, 40)
    c -= Pos(0, ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(20.0, 100)
    for yy in (ROLL_Y - CLEVIS_EAR + 6, ROLL_Y + CLEVIS_EAR - 6):
        c -= Pos((x0 + x1) / 2, yy, 0) * Cylinder(PINCH["d"] / 2 + 0.25, 20, align=MIN)
    return c


def yaw_shaft() -> Part:
    """D19 (loop 44, after Astra review #2b finding 1): Ø40 tube with the 40T pulley and the clock-spring hub printed on
    it in one piece (no keyed joint). The bearings slide on from the two ends (A from the top, B from the bottom), then
    the whole sub-assembly (+ spring ring and ribbon) slides sideways into the plates' U-slots, which the two
    yaw_seat_caps close. The flange is a separate part bolted to the tube's lower end (yaw_flange)."""
    z0 = FL_TOP - FL_REC                                                       # tube end = the flange's recess floor
    h = P1[1] + 2 - z0
    s = Pos(0, ROLL_Y, z0) * (Cylinder(20, h, align=MIN) - Cylinder(YF["bore_r"], 200, align=MIN))
    s += Pos(0, ROLL_Y, YAW_BELT - 8) * (pulley(40, 16, flange=False) - Cylinder(YF["bore_r"], 16, align=MIN))
    s += Pos(0, ROLL_Y, P2[1]) * (Cylinder(21.6, 1.0, align=MIN) - Cylinder(YF["bore_r"], 1.0, align=MIN))
    s += Pos(0, ROLL_Y, P2[1] + 1.0) * (Cylinder(YAW_SPR[0], YAW_BELT - 8 - P2[1] - 1.0, align=MIN) - Cylinder(YF["bore_r"], 30, align=MIN))
    s -= Pos(0, ROLL_Y, z0) * Rot(0, 0, 30) * _hex(YF["hex_af"] + 0.4, YF["hex_h"] + 0.2)   # torque socket for the flange's hex spigot
    t = YAW_TIE
    top = P1[1] + 2
    s -= Pos(0, ROLL_Y, top - t["cb_depth"]) * Cylinder(t["cb_r"], 10, align=MIN)      # C1: counterbore for the clamp ring + heads
    for a in _yaw_tie_angles():                                                   # C1: 6 × Ø3.4 through the wall (no inserts)
        s -= Pos(t["r"] * math.cos(a), ROLL_Y + t["r"] * math.sin(a), z0 - 1) * Cylinder(t["hole"] / 2, top - z0 + 2, align=MIN)
    return s


# Yaw flange ↔ tube joint (Astra #2b finding 2, D20 payload): a hollow hex spigot on the flange (AF 24 × 8, Ø16 ribbon bore)
# in a hex socket in the tube end carries the torque by form.
# C1 (Fable review 2026-11-27): the bending and hanging load were on 6 × M3 into heat-set inserts in the tube end, in
# pull-out. Now the flange is **6061-T6** (machined, ESTIMATE $25) and the joint is a tie: 6 × M3 × 45 class 12.9 socket
# caps (ISO 4762) go down from the tube's top end, heads on a steel clamp ring (r 8.5–16.8 × 2) in a Ø34 × 5.5 counterbore
# flush under the top, through Ø3.4 holes in the tube wall (bore Ø16 for the ribbon, wall r 8–20) and the hex-socket
# void, into M3 threads tapped in the aluminium spigot + plate (≈ 10.8 mm engaged, Loctite 243). The whole printed tube
# is clamped in compression between the ring and the flange, so the hanging load goes flange → screws → ring → tube
# (compression) → shoulder → bearing B, and the tube's bending at bearing B is prestressed. Screws at the hex corners
# (r 11, 30° + k·60°). Tightened from above before the yaw block goes onto the roll output. Capacity: rnd/joints.py.
YF = dict(hex_af=24.0, hex_h=8.0, bore_r=8.0, spigot_r_in=8.0)
YAW_TIE = dict(n=6, r=11.0, ang0=30.0, d=3.0, pitch=0.5, As=5.03, hole=3.4, tap=2.5, grade="12.9", proof=970.0, preload=1000.0,
               creep=0.75, ring=(8.5, 16.8, 2.0), cb_r=17.0, head_h=3.0, top_gap=0.5, L=45.0, tau_flange=207.0)
YAW_TIE["cb_depth"] = YAW_TIE["ring"][2] + YAW_TIE["head_h"] + YAW_TIE["top_gap"]    # 5.5 mm


def _yaw_tie_angles():
    return [math.radians(YAW_TIE["ang0"] + 360.0 * k / YAW_TIE["n"]) for k in range(YAW_TIE["n"])]


def yaw_tie_levels():
    """z of the ring seat, the under-head plane, the screw tip and the thread engaged in the flange (mm)."""
    t = YAW_TIE
    top = P1[1] + 2
    seat = top - t["cb_depth"]
    head = seat + t["ring"][2]
    tip = head - t["L"]
    spigot_top = FL_TOP - FL_REC + YF["hex_h"]
    return dict(seat=seat, head=head, tip=tip, engaged=spigot_top - tip, tip_to_face=tip + YAW_DROP)


def yaw_tie_ring() -> Part:
    """C1: steel clamp ring (laser-cut S235, r 8.5–16.8 × 2, 6 × Ø3.4) under the 6 screw heads, at the counterbore floor."""
    r0, r1, th = YAW_TIE["ring"]
    z = yaw_tie_levels()["seat"]
    p = Pos(0, ROLL_Y, z) * (Cylinder(r1, th, align=MIN) - Cylinder(r0, th, align=MIN))
    for a in _yaw_tie_angles():
        p -= Pos(YAW_TIE["r"] * math.cos(a), ROLL_Y + YAW_TIE["r"] * math.sin(a), z) * Cylinder(YAW_TIE["hole"] / 2, th, align=MIN)
    return p


def yaw_flange() -> Part:
    """Upper-arm flange (6061-T6 since C1; the upper bone bolts to it inside the stop slot), on the yaw tube's lower end: a
    Ø64 × FL_REC recess in its top clears the bearing-B boss; in it, the hollow hex spigot goes up into the tube's socket
    (torque); 6 × M3 tapped holes at the hex corners take the YAW_TIE screws from the tube's top (bending, axial). Hard-stop
    slot (blind, 3.5 mm deep in its top) for the fixed Ø6 stop pin: software ±150°, hard ±155°."""
    z0 = -YAW_DROP
    f = Pos(0, ROLL_Y, z0) * Cylinder(FL_R, FL_T, align=MIN)
    f -= Pos(0, ROLL_Y, z0 + FL_T - FL_REC) * Cylinder(BOSS[1] + 2.0, FL_REC + 1, align=MIN)
    zr = z0 + FL_T - FL_REC
    f += Pos(0, ROLL_Y, zr) * Rot(0, 0, 30) * _hex(YF["hex_af"] - 0.2, YF["hex_h"])
    f -= Pos(0, ROLL_Y, z0 - 1) * Cylinder(YF["spigot_r_in"], 50, align=MIN)          # ribbon / cable bore Ø16
    f -= Pos(0, ROLL_Y, z0 + 1.5) * _arc_slot(STOP_PIN_R, 3.3, STOP_PIN_AZ, YAW_STOP_DEG, 4.0)
    tip = yaw_tie_levels()["tip"]
    for a in _yaw_tie_angles():
        f -= Pos(YAW_TIE["r"] * math.cos(a), ROLL_Y + YAW_TIE["r"] * math.sin(a), tip) * Cylinder(YAW_TIE["tap"] / 2, zr + YF["hex_h"] - tip, align=MIN)
    return f


def _arc_slot(r, hw, az, half, h):
    """Annular slot of half-width hw at radius r, ±half degrees about azimuth az (round ends), height h, from z = 0."""
    from build123d import Polygon, extrude
    ring = Cylinder(r + hw, h, align=MIN) - Cylinder(r - hw, h, align=MIN)
    keep = [(0.0, 0.0)] + [(3 * r * math.cos(math.radians(az + 180 + a)), 3 * r * math.sin(math.radians(az + 180 + a))) for a in range(-(180 - half), 180 - half + 1, 5)]
    gap = extrude(Polygon(*keep, align=None), h)                      # the solid sector the slot must NOT cut (around az + 180)
    ring -= gap
    for a in (-half, half):
        t = math.radians(az + a)
        ring += Pos(r * math.cos(t), r * math.sin(t), 0) * Cylinder(hw, h, align=MIN)
    return ring


def yaw_pulley() -> Part:
    """40T HTD 5M on the belt plane, hex bore, with a Ø45 hub below it that is the clock spring's rotating inner wall
    (r 22.5 ≥ 10 × the Ø2.2 USB cable's OD); the hub stands on bearing B's inner race."""
    p = Pos(0, 0, YAW_BELT - 8) * pulley(40, 16, flange=False)
    p += Pos(0, 0, P2[1]) * Cylinder(21.6, 1.0, align=MIN) + Pos(0, 0, P2[1] + 1.0) * Cylinder(YAW_SPR[0], YAW_BELT - 8 - P2[1] - 1.0, align=MIN)
    p -= Pos(0, 0, P2[1]) * _hex(HEX_AF + 0.4, 40)
    return Pos(0, ROLL_Y, 0) * p


def yaw_spring_ring() -> Part:
    """Static outer wall of the yaw clock spring, screwed to the bottom plate; ribbon exit toward +X (cable comes down
    the +X clevis arm and in between the plates)."""
    r1 = YAW_SPR[1]
    top = RING_TOP
    g = Pos(0, ROLL_Y, P2[1]) * (Cylinder(r1 + 1.5, top - P2[1], align=MIN) - Cylinder(r1, top - P2[1], align=MIN))   # 1.5 mm wall (r_out 42.5)
    g -= Pos(r1 + 1, ROLL_Y, P2[1] + 0.5) * Box(4, 10, CAV_H, align=MIN)
    return g


YAW_STOP_DEG = 155                    # hard stops (software limit ±150°, rnd/twin.py JR); loop 40, was ±75°
STOP_PIN_R, STOP_PIN_AZ = 38.0, 180.0 # stop pin radius and azimuth at yaw 0 (under the bottom plate, toward −X)


def yaw_stop_pin() -> Part:
    """Ø6 hardened dowel pressed through the 3.8 mm pocket ceiling of the bottom yaw plate (top flush with the cavity floor),
    across the 2.3 mm gap and 3.2 mm into the flange's blind arc slot. A fault drive into the stop at the yaw's 20.9 N·m
    joint peak puts ≈ 550 N on the pin: ≈ 29 MPa bearing over 3.2 mm of printed flange and ≈ 24 MPa in the plate seat
    (fault-only, software limit ±150°; wave 1: 32 / 19 MPa); ≈ 2.1 N·m bending, ≈ 100 MPa in the pin."""
    t = math.radians(STOP_PIN_AZ)
    zb = FL_TOP - 3.2
    return Pos(STOP_PIN_R * math.cos(t), ROLL_Y + STOP_PIN_R * math.sin(t), zb) * Cylinder(3, P2[1] - zb, align=MIN)


CAP_GAP = (12.0, 62.0, 62.0)                # can rear → cap, can axis → cap top, roll axis → cap outboard (≥ 2 mm to the can)


def shoulder_cap() -> Part:
    """Looks pass: rounded cap over the roll cartridge and the top of the roll link (pitch body; open underneath).
    It stays clear of everything that rolls: the clevis stays within 37 mm of the roll axis and the cap starts 62 mm above it."""
    from .soft import soft_shell
    # D24: the bolted roll cartridge (Ø86.6, rear at XF − ZC) has no KX collar (r 52 lugs) or torque fork behind it, so
    # the cap shrinks: rear XF − ZC − CAP_GAP[0], top ROLL_C + CAP_GAP[1], outboard ROLL_Y + CAP_GAP[2] (was −113 / ROLL_C + 70 / ROLL_Y + 70)
    x0, x1 = XF - ZC - CAP_GAP[0], 33.0
    z1 = ROLL_C + CAP_GAP[1]
    y1 = ROLL_Y + CAP_GAP[2]
    xc = (x0 + x1) / 2
    c = Pos(xc, (10 + y1) / 2, (38 + z1) / 2) * soft_shell(x1 - x0, y1 - 10, z1 - 38, 42, 30, 2.0)
    c -= Pos(xc, (10 + y1) / 2, 38) * Box(x1 - x0 + 6, y1, 4)                 # open bottom (rear part)
    c -= Pos(14, (10 + y1) / 2, 38 + 12) * Box(40, y1, 28)                    # front part opens higher (roll pulley sweep)
    c -= Pos(-8, ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(_R_ROLL + 5, 30)      # clearance around the roll pulley
    return c - roll_sweep_envelope()


def roll_sweep_envelope(pad=3.0, step=5):
    """Loop 40 (rnd/swept.py): the space the roll output sweeps over roll −15…100° (yaw block + band, yaw cartridge +
    collar, as padded boxes rotated about the roll axis). Static covers are cut by it: the first swept check found the
    shoulder cap in the path of the yaw block and cartridge at roll 55–95° (true since loop 22, invisible to static checks).
    D24: the bolted yaw cartridge (Ø86.6 can, rear 69 mm above the top plate) replaces the KX74 collar + can + fork boxes."""
    from build123d import Axis
    from ..twin import JR
    boxes = [(-130 - pad, 56 + pad, ROLL_Y - 50 - pad, ROLL_Y + 50 + pad, P2[0] - pad, P1[1] + pad),          # plates + band
             (-YAW_C - R_CAN - pad, -YAW_C + R_CAN + pad, ROLL_Y - R_CAN - pad, ROLL_Y + R_CAN + pad, ZF - 2 - pad, ZF + ZC + pad)]   # D24 bolted yaw cartridge
    ax = Axis((0, ROLL_Y, 0), (1, 0, 0))
    env = None
    for t in list(range(JR["roll"][0] - 5, JR["roll"][1] + 6, step)):
        for x0, x1, y0, y1, z0, z1 in boxes:
            b = Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)
            b = b.rotate(ax, t)
            env = b if env is None else env + b
    return env


def yaw_band() -> Part:
    """Rounded band around the yaw-block plates (roll body)."""
    from .soft import soft_box
    x0, x1 = -YAW_C - 50, X_PL1 + 8           # loop 40: +6 at +X for the ribbon from the roll spring
    zc, hz = (P2[0] + P1[1]) / 2, P1[1] - P2[0] + 3
    b = Pos((x0 + x1) / 2, ROLL_Y, zc) * (soft_box(x1 - x0, 100, hz, 30, 6) - soft_box(x1 - x0 - 4, 96, hz + 2, 28, 0.6))
    if INLINE:                                # 45° chamfer on the inboard top edge: it passes the ferrule at roll −15°
        c = BAND_CH
        b -= Pos(0, ROLL_Y - 50, P1[1] + 1.5) * Rot(45, 0, 0) * Box(400, c * math.sqrt(2), c * math.sqrt(2))
    return b


def module():
    if INLINE:
        return module_inline()
    out = [("ferrule_m", Rot(-90, 0, 0) * ferrule(male=True), "orange"),
           ("roll_link", roll_link(), "white"), ("roll_shaft", roll_shaft(), "orange"),
           ("roll_seat_cap_A", roll_seat_cap(W1), "white"), ("roll_seat_cap_B", roll_seat_cap(W2), "white"),
           ("roll_spring_hub", roll_spring_hub(), "dark"), ("roll_spring_ring", roll_spring_ring(), "dark"),
           ("yaw_block", clevis_and_yaw_block(), "white"), ("roll_clevis_cap_in", roll_clevis_cap(0), "white"), ("roll_clevis_cap_out", roll_clevis_cap(1), "white"), ("yaw_shaft", yaw_shaft(), "orange"), ("yaw_flange", yaw_flange() + yaw_tie_ring(), "steel"),
           ("yaw_plate_bot", yaw_plate_bot(), "white"),
           ("yaw_spring_ring", yaw_spring_ring(), "dark")]
    out += [("shoulder_cap", shoulder_cap(), "white"), ("yaw_band", yaw_band(), "white")]
    out += [(f"roll_{n}", p, c) for n, p, c in _stage(ROLL_T, 18)]
    out += [(f"yaw_{n}", p, c) for n, p, c in _stage(YAW_T, 20)]
    out.append(("yaw_stop_pin", yaw_stop_pin(), "steel"))
    return out


def masses(density=1.27, split=False):
    """CAD roll-up (loop 40). Printed parts at 60–80 % fill; D13 standard parts by material (steel shaft, aluminium
    pulley, 6801 at 60 % of solid steel). split=True → (roll-body mass, yaw-stage mass) for params.
    D24 (2026-11-27): each stage is small_stage_v3 + the bolted cartridge; as in shoulder.masses the sleeve counts only by
    what it adds over the loop-10 can (nose, flange, rear cap), the Oldham drive replaces the KX74 hub + collar + coupling."""
    from .belt_stage import small_stage_v3
    st = dict((n, p) for n, p, _ in small_stage_v3(20))
    d13 = (st["shaft_12"].volume * 7.85e-3 + st["al_pulley"].volume * 2.7e-3
           + (st["brg_A_6801"].volume + st["brg_B_6801"].volume) * 7.85e-3 * 0.6) / 1e3          # one stage, kg
    sock = (st["flange_coupling"].volume * 2.7e-3 * 0.8 + st["oldham_disc"].volume * 1.41e-3 + st["kx_hub_v2"].volume * density * 0.7e-3
            + (cm.can_bolted().volume + cm.can_cap_bolted().volume - cm.can().volume) * density * 0.8e-3) / 1e3 + 0.006   # + 6 × M4 csk, inserts
    pr = lambda v, f: v * f / 1e6 * density
    if INLINE:      # the GIM8108-36 itself (1.0 kg) is the roll actuator mass in params.arm.joints, not counted here
        roll = {"roll link: ferrule boss, frame, motor floor, support stub, stop lugs, spring bar (printed)": pr(roll_link().volume, 0.6),
                "roll_arm_rear (printed)": pr(roll_arm_rear().volume, 0.6),
                "roll spring hub + ring (printed)": pr(roll_spring_hub().volume + roll_spring_ring().volume, 0.8),
                "61806 + 12 × M4 + 2 × M5 + end washer (ESTIMATE)": 0.045,
                "male ferrule (printed) + dowels + screws + ribbon (ESTIMATE)": pr(67213, 0.7) + 0.04}
    else:
        roll = {"roll link + seat caps + spring ring/bar (printed)": pr(roll_link().volume + roll_seat_cap(W1).volume + roll_seat_cap(W2).volume, 0.6) + pr(roll_spring_ring().volume, 0.8),
                "roll shaft + 80T (one piece, D19) + spring hub": pr(roll_shaft().volume, 0.7) + pr(roll_spring_hub().volume, 0.8),
                "roll D13 v3 stage + Oldham + bolted can flange/nose/cap": d13 + sock, "2 × 61808 (ESTIMATE)": 0.04, "male ferrule (printed) + dowels + belt + screws + ribbon (ESTIMATE)": pr(67213, 0.7) + 0.05}
    caps = 0.0 if INLINE else roll_clevis_cap(0).volume + roll_clevis_cap(1).volume
    yaw = {"clevis + caps + yaw block plates + posts (printed)": pr(clevis_and_yaw_block().volume + caps + yaw_plate_bot().volume, 0.6),
           "yaw shaft + 40T + spring hub (one piece), spring ring (printed)": pr(yaw_shaft().volume, 0.7) + pr(yaw_spring_ring().volume, 0.8),
           "C1 yaw flange (6061-T6) + steel clamp ring + 6 × M3 × 45 12.9": yaw_flange().volume * 2.7e-6 + yaw_tie_ring().volume * 7.85e-6 + 6 * (5.03 * 45 + 45) * 7.85e-6,
           "yaw D13 v3 stage + Oldham + bolted can flange/nose/cap": d13 + sock, "2 × 61808 + stop pin + belt + screws + ribbon (ESTIMATE)": 0.075}
    if split:
        return sum(roll.values()), sum(yaw.values())
    return {**{"roll: " + k: v for k, v in roll.items()}, **{"yaw: " + k: v for k, v in yaw.items()}}


def drive_capacity():
    """Shaft → link torque paths of the roll and yaw (Astra #2b finding 2) vs the joint peak torque, as rows
    (joint, path, capacity, demand, ratio, basis). Hand calculations; preloads, friction and insert pull-out are ESTIMATEs
    until coupon C2-style tests (slip torque, pull-out) on the printed parts."""
    from ..params import BASELINE as d, G, act
    a = d.arm
    rows = []
    T_yaw = act(a.joints["upper_yaw"][0])["peak"] * a.joints["upper_yaw"][1] * 0.95          # 20.9 (rnd/basket.py capacity, 95 % belt)
    if INLINE:
        m = act("GIM8108-36")
        M, A, o = MOT36, ADAPT, OUT_CLAMP
        r0, r1 = M["out_pcd"] / 2000, A["pcd"] / 2000
        # (a) motor output → metal adapter. M1 (Fable review 2026-11-27): wave 2 counted 26 N·m of friction from a 4 kN preload
        # that 3 mm of M4 in aluminium can't hold. Now the dowels alone carry the torque (rnd/joints.py roll_rows, same numbers)
        t_sh = M["dowel_n"] * math.pi * 2.0 ** 2 * 450.0 * r0                             # ISO 8734 Ø4: min double shear 11.3 kN → single 5.65 kN
        t_bb = M["dowel_n"] * M["dowel_d"] * M["dowel_l"] * 240.0 * r0                   # bearing yield in the motor's output, 4 mm deep (ESTIMATE 240 MPa)
        t_m4 = M["out_n"] * 1500.0 * 0.15 * r0                                           # bonus, not counted: 1.5 kN per M4 (3.0 mm of thread in Al)
        basis0 = (f"{M['dowel_n']} × Ø4 m6 hardened dowels in H7: shear {t_sh:.0f} N·m (ISO 8734), bearing in the motor's output {t_bb:.0f} N·m (240 MPa bearing yield, "
                  f"4 mm deep, ESTIMATE: cast A380 / 6061-T6; governs); the 3 × M4 × 10 csk friction ({t_m4:.0f} N·m at an honest 1.5 kN) is not counted")
        rows.append(("roll (inline)", "motor output ↔ 7075 adapter (dowels only) vs the drive limit (35.1)", min(t_sh, t_bb), m["rated"], basis0))
        rows.append(("roll (inline)", "motor output ↔ 7075 adapter (dowels only) vs stall", min(t_sh, t_bb), m["peak"], basis0 + f"; stall {m['peak']:.1f} N·m (actuators.yaml)"))
        # (b) metal adapter → printed clevis arm
        F = A["n"] * o["preload"] * o["creep"]
        t_fr = o["mu"] * F * r1
        L = o["arm_t"] - o["ring"][2] - o["head_h"]
        t_br = A["n"] * A["d"] * L * 12.0 * r1                                           # fitted holes at the 12 MPa across-layer allowable
        basis = (f"{A['n']} × M5 on PCD {A['pcd']:.0f}, {o['preload']:.0f} N preload (≈ 5 MPa on the PETG under the steel ring) × {o['creep']} creep, μ {o['mu']} PETG/Al (ESTIMATE)")
        rows.append(("roll (inline)", "adapter ↔ clevis arm, friction vs the drive limit (35.1)", t_fr, m["rated"], basis))
        rows.append(("roll (inline)", "adapter ↔ clevis arm, friction + fitted-bolt bearing vs stall", t_fr + t_br, m["peak"],
                     f"+ {A['n']} × Ø5 × {L:.1f} mm at 12 MPa (fea.py across-layer allowable) = {t_br:.0f} N·m after ≤ 0.1 mm slip; stall only on a fault"))
    else:
        p = PINCH
        T_roll = act(a.joints["shoulder_roll"][0])["peak"] * a.joints["shoulder_roll"][1] * 0.95   # 46.4 (rnd/basket.py capacity)
        F = p["n_per_arm"] * p["preload"] * p["creep"]
        t_arm = p["mu"] * math.pi * F * 0.020                                                   # split clamp, uniform pressure: N = π F
        press = [F / (40.0 * (x1 - x0)) for x0, x1 in CLEVIS_X]
        rows.append(("roll (belt)", "roll shaft ↔ clevis, 2 pinch clamps (friction)", 2 * t_arm, T_roll,
                     f"per arm {p['n_per_arm']} × M5 into heat-set inserts, {p['preload']:.0f} N (ESTIMATE ≈ 50 % of M5 insert pull-out in PETG-CF) × {p['creep']} creep, "
                     f"μ {p['mu']} PETG-CF/PETG-CF (ESTIMATE); T = μ·π·F·r per arm; clamp pressure {press[0]:.1f} / {press[1]:.1f} MPa on a solid shaft end; demand = GIM6010-8 peak × 4.44 × 0.95"))
    # yaw flange ↔ tube (both drives): hex spigot (form) for torque; C1: the YAW_TIE clamp for bending and the hanging load
    y = YF
    rc, strip = y["hex_af"] / 2000, 3.0
    t_hex = 6 * 25.0 * strip * y["hex_h"] * rc                                                  # flats bear in-layer (25 MPa)
    ro, ri = y["hex_af"] / 2 * 1.05, y["spigot_r_in"]
    rm, tw = (ro + ri) / 2, ro - ri
    t_root = 12.0 * 2 * math.pi * rm ** 2 * tw / 1000                                            # root torsion on a layer plane (12 MPa)
    t = YAW_TIE
    t_cl = t["n"] * t["preload"] * t["creep"] * 0.30 * t["r"] / 1000
    rows.append(("yaw", "tube ↔ flange, hex spigot AF 24 × 8 (form)", min(t_hex, t_root), T_yaw,
                 f"flats {t_hex:.0f} N·m (3 mm strips at 25 MPa in-layer), spigot root {t_root:.0f} N·m (torsion at 12 MPa across layers); "
                 f"+ {t_cl:.0f} N·m clamp friction from the YAW_TIE (μ 0.3) not counted; demand = GIM6010-8 peak × 2 × 0.95"))
    from .. import joints
    J = joints.joint_models()["yaw flange ↔ tube"]
    try:
        import json as _json
        dem = _json.loads((joints.DOCS / "joints.json").read_text())
        dem = max(r["M"] for r in dem["joints"] if r["joint"] == "yaw flange ↔ tube" and r["case"] in joints.SERVICE)
        src = "rnd/joints.py: the twin's worst basket/hug wrench × 1.5"
    except (OSError, ValueError, KeyError):
        dem, src = 20.7 * 1.5, "20.7 N·m × 1.5, rnd/basket.py worst shoulder-pitch torque (run python -m rnd.joints for the twin value)"
    mg = min(m.m_gap(0.0) for m in J["models"])
    rows.append(("yaw", "tube ↔ flange, C1 tie (6 × M3 12.9 into the 6061 flange): moment before the joint gaps vs the service bending", mg, dem,
                 f"clamp model (rnd/joints.py): {t['n']} × {t['preload'] / 1e3:.1f} kN × {t['creep']} creep on the r 14.1–20 tube end; demand {src}; "
                 "fault cases (140 N bump, stalls) in docs/rnd/joints.md"))
    return [dict(joint=j, path=pth, cap=round(c, 1), demand=round(dm, 1), ratio=round(c / dm, 2), basis=b) for j, pth, c, dm, b in rows]
