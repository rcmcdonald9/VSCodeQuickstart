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
from .belt_stage import V3 as _V3, bolted_stage_v4, wall_a_cuts_v4
INLINE = _D.arm.roll_drive == "inline"
EAR_SITE = "yaw"            # D24 v2 flange-ear pattern (cartridge_m.EAR_SITES) of the yaw stage (and the belt-roll fallback), read by rnd/assembly_seq.py
DRIVE = "direct"            # D26 (2026-11-27): both GIM6010 stages here are direct stages v4 (no Oldham), fixed centres (rnd/belts.py FIXED_CENTRE)
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
REAR_ANG0 = 0.0                                                    # first of the 8 rear M4: set below from the hard-stop geometry (two stop bushes sit on two of them)
MOT36["h"] = MOT36["body_h"] + sum(h for _, h in MOT36["step"])   # body rear face → output step face (44.5); + the driver = 57
CUP_H = MOT36["drv_h"] + 0.5                                       # roll-link cup ring around the driver section
CUP_R = MOT36["drv_d"] / 2 + 0.1                                   # Neon 6c: the cup bore is the motor's radial pilot (was + 0.5): Ø75.2 printed, sanded to a slip fit
# M1 (Fable review 2026-11-27): the housing's M4 are tapped only 3.5 deep; wave 2's M4 × 18 under a 3.5 mm counterbore
# engaged 2.5 ± 0.55 mm. Wave 3 shimmed it to 3.0 ± 0.2 but still clamped 13.9 mm of PETG at 0.5 kN (4 µm of screw
# stretch): any creep of the print emptied the joint (Neon review 2026-11-27, finding 6b).
# Now a metal clamp: a 3 mm 7075-T6 rear plate (ring r 35–48, waterjet) on the roll link's −X face, 6 × M4 × 25 ISO 10642
# 10.9 countersunk flush in it and 2 × M4 × 25 ISO 4762 12.9 inside the two stop bushes, each through a steel sleeve
# (Ø6 / Ø4.3, or the bush's Ø7 tail) that runs through the printed floor + cup ring onto the motor's rear face. The stack
# head → plate (or bush) → sleeve → motor is all metal, so the 1.6 kN preload only loses its embedding; the print is
# sandwiched between the plate and the motor face (sleeves 0.05 under the print: located, lightly squeezed, not the
# spring). Engagement 25 − 22 = 3.0 ± 0.47 from stock tolerances (2.53–3.47 in the 3.5 mm holes): no shim picking.
# The whole motor stack moves 3 mm toward +X (the plate takes the 2.5 mm gap's place). Rows: rnd/joints.py.
REAR_SCREW = dict(size="M4 × 25 ISO 10642 10.9 (6) / ISO 4762 12.9 in the stop bushes (2)", n=8, L=25.0, L_tol=0.42, plate_t=3.0, plate_tol=0.05,
                  sleeve_od=6.0, sleeve_id=4.3, sleeve_tol=0.02, sleeve_fit=-0.05, preload=1600.0, mu=0.15, tap=MOT36["rear_tap"], floor_t=6.0)
REAR_SCREW["grip"] = REAR_SCREW["plate_t"] + REAR_SCREW["floor_t"] + CUP_H         # 22: csk head top → the motor's rear face
REAR_SCREW["engage"] = REAR_SCREW["L"] - REAR_SCREW["grip"]                         # 3.0 mm into the 3.5 mm tapped holes
PLATE_R = (35.0, 48.0)                                             # rear plate ring radii (inside: the 61806 stub's floor stays printed)
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
ROLL_C, YAW_C = 103.3, 81.18         # 80T:18T roll on a stock 475 mm belt; yaw: the exact centre for a stock 315-5M belt (80.93, Neon #2 F7) + 0.25 mm of fixed-centre preload (D26, rnd/belts.py FIXED_CENTRE)
# D24 (2026-11-27): both stages are belt_stage.small_stage_v3 with the bolted cartridge (D24 v2, Fable C3: M4 ear bolts from
# the cartridge side into nuts on wall A's pulley face, site EAR_SITE; centring lip in the wall's counterbore, Oldham drive). Stage frame: z = 0 is the
# flange plane = the cartridge-side face of wall A; the bearings, pulley and belt plane sit where the KX74 V2 stack had them.
JA, JB = _V3["jA"], _V3["jB"]                          # D13 bearing spans (stage frame, below the flange plane: negative z)
PUL_C = -_V3["belt_c"]                                 # 16.5: small-pulley belt centre below the flange plane
ZC = cm.bolted_levels()["rear"]                        # flange plane → can rear (69.1; the KX74 cartridge stood 80.4 above wall A)
R_CAN = cm.MOTOR["od"] / 2 + cm.WALL + 0.3               # bolted can sleeve radius (43.3)


def _stage(T, teeth=20):
    """D26: bolted cartridge + direct stage v4 (belt_stage.bolted_stage_v4: stub on the motor, 6001 in wall B), mapped by T."""
    return bolted_stage_v4(T, teeth, site=EAR_SITE)


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
            w -= ROLL_T * wall_a_cuts_v4(None, EAR_SITE)                          # D26: Ø40.6 pulley hole, spigot counterbore, ear bolts + nut pockets
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
    if INLINE:                                                        # M3: AS5048A magnet web (rnd/parts/encoder.py)
        from .encoder import roll_hub_web
        h += roll_hub_web()
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
# M3 (Fable, 2026-11-27): the yaw AS5048A board sits on pads on the top plate over the tube end (rnd/parts/encoder.py), so
# the yaw tube's top end is trimmed from 2.0 mm above the plate's top face to 0.5 mm under it (the yaw 61808 A, 7 mm in the
# 6 mm plate, keeps 6.5 mm of tube under its inner race).
YAW_TOP = P1[1] - 0.5
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
#   fork | 2.5 mm | 7075 rear plate (3 mm, Neon review) + stop bushes | roll-link floor (6 mm; 8 × M4 through plate + sleeves
#   into the motor's rear face) | GIM8108-36 housing (static) | output flange
#   | +X clevis arm (fused to the yaw block, 6 × M4 into the flange) | roll clock-spring cassette (hub on the arm, ring on
#   the roll link's bar, as in the belt stage).
# Two-point roll support: the motor's output bearing (+X) and the 61806 (−X), 68 mm apart (belt: 2 × 61808, 39 mm apart);
# the motor's output bearing then carries ≈ 19 N·m / 0.068 ≈ 280 N radial in the 1 kg workspace, ≈ 0.6 kN at the
# basket × 1.5 and 2.3 kN at a stall (rnd/joints.py, from the twin; rating not published: confirm with the seller). Neon
# review 6c: the 61806 floats axially and is bonded in its seat in situ (SEAT_GAP, ROLL_SUPPORT below). Assembly: motor
# into the cup pilot, rear plate + sleeves + stop bushes, 8 × M4 from −X → 61806 onto the stub, DIN 471 ring → arm over
# it from −X (race into the oversize seat, Loctite 660, cured with the arm bolted at mid-travel) → yaw block (+ yaw cartridge) up from below: its arm slides down the flange face,
# 6 × M4 into the flange from +X, roll_arm_rear's foot 2 × M5 into the top plate (the torque pin rises into the fork
# slot) → spring hub on the arm, ribbon, spring ring onto the bar. No pulley, belt or D13 stage, so R14 has nothing to
# block (rnd/assembly_seq.py checks the 61806 slide-on).
# Roll travel: software −15…100° (twin.JR); hard stops at −16.5° / 104°. M1 (Fable review 2026-11-27): the stops were two
# printed Ø8 lugs (≈ 800 N at the 35 N·m drive limit, ≈ 16 MPa vs the 12 MPa allowable; 2.3 kN at stall); wave 3's Ø10
# dowels at r 43 had no seat wall (Neon 6d). Now two steel stop bushes on two of the rear M4 (STOP_BUSH below) meet
# 20 × 8 × 1.5 stainless wear strips bonded flush into roll_arm_rear's edges. Rows: rnd/joints.py.
# ---------------------------------------------------------------------------------------------------------------------
BRG_R = dict(d=30.0, D=42.0, B=7.0)    # 61806, rear roll support
STUB_RELIEF = 0.5                      # the Ø30 stub's −X end stops this far inside the 61806's −X face (yaw can clearance, rnd.swept)
# +X clevis arm ↔ metal output adapter (Astra #2b finding 2; the motor's own 3 × M4 on PCD 28.5 are too small to go
# straight into a print): 8 × M5 low-head (DIN 7984) 10.9 from +X through the arm into the adapter's threads, heads on a
# Ø80/Ø52 × 2 steel clamp ring in a counterbore (≈ 5 MPa on the PETG at the design preload); fitted Ø5.1 holes give a
# bearing path behind the friction. Capacities: drive_capacity().
# Neon review (same physics as findings 3 / 6b): 8.5 mm of PETG under the ring was the clamped member; with the creep
# model (rnd/joints.py life_preload, φ 2 + embedding) ≈ 12 % of the 1.5 kN is left, so the friction path is gone and the
# arm would slip onto the fitted holes (backlash). Now each M5 runs in a steel crush sleeve (Ø8 / Ø5.3, 42CrMo4+QT, a
# light press fit in the print) from the ring to the adapter face: the clamp is ring → sleeve → adapter in metal, and
# the print takes the torque in bearing on the sleeves (no slip clearance). Capacities: drive_capacity().
OUT_CLAMP = dict(arm_t=14.0, ring=(26.0, 40.0, 2.0), head_h=3.5, preload=1500.0, creep=0.75, mu=0.25, sleeve=(8.0, 5.3), mu_metal=0.15)
R_ARM = 42.0                           # clevis-arm boss radius about the roll axis (covers the Ø76 adapter and the ring)
R_ARM_REAR = 35.5                      # roll_arm_rear boss radius (61806 seat): 1.5 mm inside the stop bushes (r 42 − 5)
# Hard stops (Neon review 2026-11-27, finding 6d): wave 3's Ø10 pins at r 43 were to be pressed into the floor + the cup
# ring, but the cup ring is r 38–48: no wall either side of a Ø10 seat, and 2.4 kN at stall on it (they were not drawn;
# printed lugs stood in). Now each stop is a steel bush (42CrMo4+QT, STOP_BUSH) on one of the 8 rear M4: Ø10 standing
# 8 mm proud of the rear plate, fitted in the plate, its Ø7 tail through the print to the motor face, clamped by its M4
# (head inside its Ø7.2 bore). The stall load goes bush → plate → the countersunk screws → the motor housing: the
# housing ↔ output loop closes in metal. Putting both bushes on the screw circle (r 42) needs them 5 screw pitches
# apart: the rear arm's half-width (26 → ARM_HW) and REAR_ANG0 follow from the contact angles. Rows: rnd/joints.py.
STOP_R, STOP_LUG_R = MOT36["rear_pcd"] / 2, 5.0
STOP_BUSH = dict(d=10.0, bore=7.2, d_thru=7.0, bore_lo=4.3, proud=8.0, a=4.0, mat="42CrMo4+QT", Rp=750.0, plate_Rp=435.0, strip=(20.0, 8.0, 1.5))
STOP_AT = (ROLL_MIN - 1.5, 100 + 4)    # hard-stop contact angles (deg). −16.5°: at −19° the yaw band ran 2.5 mm into the arm-side ferrule (contact from ≈ −17°, ferrule_sweep_gap)
STOP_PITCHES = 5                       # screw pitches between the two stop bushes
_DSTOP = (STOP_PITCHES * 360.0 / MOT36["rear_n"] - (STOP_AT[1] - STOP_AT[0])) / 2      # 52.25°: bush centre ↔ contact angle
ARM_HW = round(STOP_R * math.sin(math.radians(_DSTOP)) - STOP_LUG_R, 2)              # 28.22: roll_arm_rear half-width (edge ↔ bush contact)
REAR_ANG0 = math.radians((STOP_AT[0] - _DSTOP - 90.0) % (360.0 / MOT36["rear_n"]))     # 21.25°: screws (y, z) = r(cos a, sin a), roll angle = a + 90°
STOP_K = tuple(k for k in range(MOT36["rear_n"])
               if min(abs((math.degrees(2 * math.pi * k / MOT36["rear_n"] + REAR_ANG0) + 90 - c + 180) % 360 - 180) for c in (STOP_AT[0] - _DSTOP, STOP_AT[1] + _DSTOP)) < 0.01)
# R17 safe power-off roll latch (Astra review #3 finding 1; rnd/power_off.py option O2; params.arm.roll_guard = "latch",
# PIP_ROLL_GUARD=latch; default off). A spring-applied pin stops roll_arm_rear's inboard edge (the edge that meets the
# −16.5° bush) at LATCH_AT whenever its 24 V pull solenoid is off; sw/safety.py energises it only while the supervisor is
# healthy and nobody is near. Head Ø10 (ISO 8734-style hardened dowel turned to a Ø6 stem) guided by its hole in the
# 7075 rear plate and a 42CrMo4 sleeve (Ø11.5 / Ø10.05) in the roll-link floor behind it (the cup ring's Ø75.2 driver pilot
# leaves no room for the sleeve further in); engaged it stands LATCH["proud"] proud of the plate (5 of the arm's 8 mm),
# retracted it sits 0.5 mm behind the plate face; the stem runs in the cup ring (r 38.5–44.5, inside the Ø75.2 driver pilot's 37.6 and the Ø96 body) to a
# compression spring and a Ø1 steel pull cable (PTFE liner, through the side block) to the solenoid (JF-0730B class, 10 mm
# stroke, ESTIMATE) in the side block. Placement: same edge as the −16.5° bush, at radius LATCH["r"]; the clearances are
# latch_geometry(). Load path: the sleeve's far end bears on print, so it carries ≈ 0.4 kN (12 MPa) until a metal path to
# the motor face is detailed (open); sw/safety.py caps the roll torque at LATCH_T_MAX while the pin may be in, and a PU tip
# on the pin keeps a ≤ 2.5 kg unpowered fall onto it near that (power_off.md). Below LATCH_AT the arm's edge covers the pin
# (it rests on the arm face until the arm next rises past LATCH_AT: power_off.md, the O2 residual).
LATCH_AT = 5.0                          # deg: roll angle where the arm's edge meets the pin (= twin.JR_PEOPLE_NEAR roll lower bound)
LATCH = dict(r=41.5, d=10.0, sleeve_od=11.5, sleeve_id=10.05, stem_d=6.0, proud=7.5, stroke=8.0, head_l=8.5, stem_l=10.0,
             spring_n=(4.0, 8.0), mass=0.045, solenoid="24 V pull solenoid, 10 mm stroke (JF-0730B class, ESTIMATE)")
GUARD = getattr(_D.arm, "roll_guard", "none") if INLINE else "none"


def latch_angle():
    """Pin-centre angle (deg, roll convention): the arm edge (±ARM_HW) touches a Ø d pin at radius r when sin Δ = (ARM_HW + d/2)/r."""
    return LATCH_AT - math.degrees(math.asin((ARM_HW + LATCH["d"] / 2) / LATCH["r"]))


def latch_geometry():
    """Clearances of the latch sleeve / stem to the neighbouring rear screws and stop bushes, the plate ring, the driver
    pilot and the motor body (mm), and the roll range over which the arm covers the pin (deg)."""
    a = latch_angle()
    py, pz = _ang(a, LATCH["r"])
    gaps = {}
    for k in range(MOT36["rear_n"]):
        t = 2 * math.pi * k / MOT36["rear_n"] + REAR_ANG0
        y, z = ROLL_Y + STOP_R * math.cos(t), STOP_R * math.sin(t)
        d = math.hypot(y - py, z - pz)
        r_n = (STOP_BUSH["d"] if k in STOP_K else REAR_SCREW["sleeve_od"]) / 2      # in the plate: the bush head Ø10 / the screw sleeve Ø6
        gaps[k] = d - LATCH["sleeve_od"] / 2 - r_n
    so = LATCH["sleeve_od"] / 2
    cover = []
    for th in range(int(STOP_AT[0]), int(STOP_AT[1]) + 1):
        dd = LATCH["r"] * math.sin(math.radians(abs(a - th)))
        if abs(a - th) < 90 and dd < ARM_HW + LATCH["d"] / 2 - 1e-6:
            cover.append(th)
    ph = LATCH["d"] / 2 + 0.05
    return dict(angle=a, y=py, z=pz, nearest=min(gaps.values()), to_plate_in=LATCH["r"] - ph - PLATE_R[0], to_plate_out=PLATE_R[1] - LATCH["r"] - ph,
                to_floor_edge=R_MOT - LATCH["r"] - so,
                to_pilot=LATCH["r"] - LATCH["stem_d"] / 2 - CUP_R, to_body=R_MOT - LATCH["r"] - LATCH["stem_d"] / 2,
                covered=(min(cover), max(cover)) if cover else None, engage=LATCH["proud"] - (X_PLATE[0] - X_ARM_R[1]),
                head_in_floor=X_FLOOR[1] - (X_PLATE[0] + LATCH["stroke"] - LATCH["proud"] + LATCH["head_l"]))


def latch_parts(engaged=False):
    """(name, part, colour): the pin (retracted by default: the powered, nobody-near state) and its guide sleeve."""
    a = latch_angle()
    y, z = _ang(a, LATCH["r"])
    x0 = X_PLATE[0] - LATCH["proud"] + (0.0 if engaged else LATCH["stroke"])
    at = lambda x, r, L: Pos(x, y, z) * Rot(0, 90, 0) * Cylinder(r, L, align=MIN)
    pin = at(x0, LATCH["d"] / 2, LATCH["head_l"]) + at(x0 + LATCH["head_l"], LATCH["stem_d"] / 2, LATCH["stem_l"])
    sl = at(X_PLATE[1], LATCH["sleeve_od"] / 2, X_FLOOR[1] - X_PLATE[1]) - at(X_PLATE[1], LATCH["sleeve_id"] / 2, X_FLOOR[1] - X_PLATE[1])
    return [("roll_latch_pin", pin, "steel"), ("roll_latch_sleeve", sl, "steel")]


def latch_cut(plate=False):
    """The latch's seat: in the rear plate the pin's Ø10.1 guide hole; in the roll link the sleeve bore through the floor and
    the stem / spring bore in the cup ring."""
    a = latch_angle()
    y, z = _ang(a, LATCH["r"])
    at = lambda x, r, L: Pos(x, y, z) * Rot(0, 90, 0) * Cylinder(r, L, align=MIN)
    if plate:
        return at(X_PLATE[0] - 0.1, LATCH["d"] / 2 + 0.05, X_PLATE[1] - X_PLATE[0] + 0.2)
    return at(X_PLATE[1], LATCH["sleeve_od"] / 2 + 0.05, X_FLOOR[1] - X_PLATE[1]) + at(X_FLOOR[1], LATCH["stem_d"] / 2 + 1.0, CUP_H - 1.0)

# Neon review finding 6c (hyperstatic pair): the 61806 inner race slides on the printed stub (± float mm between the
# shoulder and a DIN 471 Ø30 ring: no axial preload into the motor's bearing, which locates the output axially); the
# outer race sits in a seat SEAT_GAP oversize in roll_arm_rear and is bonded there (Loctite 660) with the motor mounted
# and the arm bolted to the output at mid-travel, so the printed arm loop's run-out drops out; the cup pilot (CUP_R) and
# the stub are one print. Left over: the static offset e (ESTIMATE) against the printed support's radial stiffness k_r
# (fea.md, ESTIMATE) → the misalignment preload. Rows: rnd/joints.py.
SEAT_GAP = 0.3
ROLL_SUPPORT = dict(float=0.3, e_worst=0.25, e_rss=0.15, k_r=(1.6e3, 3.2e3))
if INLINE:
    _ycx = max(p.bounding_box().max.X for _, p, _c in _stage(YAW_T, 20) if p.bounding_box().max.Z > P1[1])
    X_ARM_R = (math.ceil((_ycx + 1.2) * 2) / 2, math.ceil((_ycx + 1.2) * 2) / 2 + 8.0)   # 1.2 mm past the yaw cartridge (its can, D24)
    X_PLATE = (X_ARM_R[1] + 2.5, X_ARM_R[1] + 2.5 + REAR_SCREW["plate_t"])      # Neon 6b/6d: the 7075 rear plate
    X_FLOOR = (X_PLATE[0], X_PLATE[1] + REAR_SCREW["floor_t"])     # floor span incl. the plate (its −X face is the plate's: the screw heads)
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
    p += Pos(xf1, ROLL_Y, 0) * Rot(0, 90, 0) * (Cylinder(R_MOT, CUP_H, align=MIN) - Cylinder(CUP_R, CUP_H, align=MIN))   # cup around the driver (its pilot)
    p += _plate_x(xf0, xf1, 8, ROLL_Y, zlo, zhi, 4)                                                             # floor ↔ frame web
    p += side_block()                                                                                          # R2b: floor + cup ↔ frame, beside the motor body
    for k in range(MOT36["rear_n"]):
        a = 2 * math.pi * k / MOT36["rear_n"] + REAR_ANG0
        y, z = ROLL_Y + MOT36["rear_pcd"] / 2 * math.cos(a), MOT36["rear_pcd"] / 2 * math.sin(a)
        p -= Pos(0, y, z) * Rot(0, 90, 0) * Cylinder(2.25, 400)
        d = STOP_BUSH["d_thru"] if k in STOP_K else REAR_SCREW["sleeve_od"]
        p -= Pos(xf0, y, z) * Rot(0, 90, 0) * Cylinder(d / 2 + 0.1, X_CUP[1] - xf0, align=MIN)   # Neon 6b: steel sleeve (or the stop bush's tail) to the motor face
    p -= Pos(xf1 - 2.5, (ROLL_Y + 8) / 2, 0) * Box(5, ROLL_Y - 8, 10)                                           # motor-lead groove to the frame
    # wave 5 (rnd.swept: 1.7 mm vs the 2 mm target): the yaw can's rear ring (r 43.3, its top 6 mm; the sleeve below is
    # r 42.55) passed 1.7 mm off the stub's −X end at every roll angle. The stub now stops STUB_RELIEF short of the 61806's
    # −X face: 2.2 mm to the ring, the inner race 0.5 proud (seated on 6.5 of its 7 mm). Open: the end washer + M5 of the
    # assembly note are not drawn, and a head has no room in front of the can (a retaining ring inside the bore would)
    x_st = W1[0] + STUB_RELIEF
    p += Pos(x_st, ROLL_Y, 0) * Rot(0, 90, 0) * (Cylinder(BRG_R["d"] / 2, xf0 - x_st, align=MIN) - Cylinder(9, xf0 - x_st, align=MIN))
    p += Pos(W1[1] + 0.3, ROLL_Y, 0) * Rot(0, 90, 0) * (Cylinder(17, xf0 - W1[1] - 0.3, align=MIN) - Cylinder(9, 20, align=MIN))   # inner-race shoulder
    x0, x1 = ROLL_SPR_X
    r1 = ROLL_SPR[1]
    p += Pos((x0 + 1 + x1 + 1.6) / 2, (12 + ROLL_Y - r1) / 2, 0) * Box(x1 + 1.6 - x0 - 1, ROLL_Y - r1 - 12 + 1, 14)   # spring bar
    if GUARD == "latch":                                                                                       # R17 roll latch seat
        p -= latch_cut()
    return p - Pos(xf0 - 1, ROLL_Y, 0) * Rot(0, 90, 0) * (Cylinder(PLATE_R[1] + 0.5, X_PLATE[1] - xf0 + 1, align=MIN) - Cylinder(PLATE_R[0] - 0.2, 20, align=MIN))   # Neon 6b: rear plate seat


# R2b (2026-11-27, rnd/stiffness.py): the 1 kg straight-out moment (16.1 N·m static about the pitch axis) reaches the
# ferrule from the floor through the 6 mm floor ↔ frame web, which it twists (a 6 × 68 strip in torsion): 3.4 mrad at the
# motor seat, 2.5 mm at the hand. The web becomes a solid block from the floor's −X face to the motor body's front face
# (x −25 … 23), between the frame (y 8) and the roll axis, z −28 … 60, hollowed to r 48.5 round the motor body (0.5 mm
# radial: the motor still slides in from +X) and to the driver bore in the cup: the floor, the cup ring and the frame
# become one deep section. Its z −28 floor is the frame's (the yaw band passes it at roll −16.5°, ferrule_sweep_gap);
# above the roll axis nothing on the roll output sweeps (the yaw plates and band stay below z −50 relative to it, roll
# −16.5 … 104°). P2 FEA (stiffness.md): roll link 2.48 → 0.80 mm at the hand, + 69 cm³ (≈ 53 g at 60 % fill).
SIDE_BLOCK = dict(zlo=-28.0, zhi=60.0, r_body=0.5)


def side_block() -> Part:
    b = SIDE_BLOCK
    x0, x1 = X_FLOOR[0], X_MOT[0] + MOT36["body_h"]
    blk = _plate_x(x0, x1, 8, ROLL_Y, b["zlo"], b["zhi"], 4)
    blk -= Pos(X_FLOOR[1], ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(CUP_R, 200, align=MIN)   # the cup's driver bore (pilot)
    blk -= Pos(X_MOT[0], ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(R_MOT + b["r_body"], 200, align=MIN)          # round the motor body
    blk -= Pos(0, 13, 0) * Rot(-90, 0, 0) * Cylinder(32.3, 12)                                              # ferrule spigot
    return blk


def _stop_lug_angles():
    """Stop-bush centre angles: roll_arm_rear's straight edges (±ARM_HW from its centre line) touch a bush of radius
    STOP_LUG_R at radius STOP_R when sin(Δ) = (ARM_HW + r) / R (= the screw angles STOP_K by construction)."""
    d = math.degrees(math.asin((ARM_HW + STOP_LUG_R) / STOP_R))
    return (STOP_AT[0] - d, STOP_AT[1] + d)


STOP_LUG_ANG = _stop_lug_angles()


def roll_arm_rear() -> Part:
    """−X clevis arm (separate print): R 35.5 boss with the 61806 seat (outer-race lip on the +X side; SEAT_GAP oversize, the
    race bonded in situ), a ±ARM_HW plate (its edges meet the stop bushes) down to a foot on the top yaw plate (2 × M5).
    (D24: the yaw cartridge's torque fork is gone; its flange screws carry the torque.)"""
    x0, x1 = X_ARM_R
    a = _plate_x(x0, x1, ROLL_Y - ARM_HW, ROLL_Y + ARM_HW, P1[1], 0, 10) + Pos(x0, ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(R_ARM_REAR, x1 - x0, align=MIN)
    a += _plate_x(x0 - 10, x1, ROLL_Y - 26, ROLL_Y + 26, P1[1], P1[1] + 5, 4)                                    # foot
    a -= Pos(x0, ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(BRG_R["D"] / 2 + SEAT_GAP, W1[1] - x0, align=MIN)          # 61806 seat (bond gap)
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
    return h + rear_metal()


def rear_metal() -> Part:
    """Neon 6b/6d: the metal bolted to the housing's rear face (one CAD part with the housing): the 7075 rear plate, the 6
    steel sleeves and the 2 stop bushes (Ø10 × 8 proud + Ø7 tail, Ø7.2 head bore)."""
    x0, x1 = X_PLATE
    pl = Pos(x0, ROLL_Y, 0) * Rot(0, 90, 0) * (Cylinder(PLATE_R[1], x1 - x0, align=MIN) - Cylinder(PLATE_R[0], x1 - x0, align=MIN))
    rs, sb = REAR_SCREW, STOP_BUSH
    for k in range(MOT36["rear_n"]):
        a = 2 * math.pi * k / MOT36["rear_n"] + REAR_ANG0
        y, z = ROLL_Y + MOT36["rear_pcd"] / 2 * math.cos(a), MOT36["rear_pcd"] / 2 * math.sin(a)
        at = lambda x, r, L: Pos(x, y, z) * Rot(0, 90, 0) * Cylinder(r, L, align=MIN)
        if k in STOP_K:
            pl -= at(x0, sb["d"] / 2, x1 - x0)
            b = at(x0 - sb["proud"], sb["d"] / 2, sb["proud"] + x1 - x0) + at(x1, sb["d_thru"] / 2, X_CUP[1] - x1)
            b -= at(x0 - sb["proud"], sb["bore"] / 2, sb["proud"]) + at(x0 - sb["proud"], sb["bore_lo"] / 2, 100)
            pl += b
        else:
            pl -= at(x0, 2.2, x1 - x0)
            pl += at(x1, rs["sleeve_od"] / 2, X_CUP[1] - x1 + rs["sleeve_fit"]) - at(x1, rs["sleeve_id"] / 2, 100)
    if GUARD == "latch":                                                       # R17 roll latch: the pin's guide hole
        pl -= latch_cut(plate=True)
    return pl


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
    if INLINE:                                                                              # M3: bridge for the roll AS5048A board
        from .encoder import roll_bridge
        ring += roll_bridge()
    return ring - Pos((x0 + 1 + x1 + 1.6) / 2, (12 + ROLL_Y - r1) / 2, 0) * Box(x1 + 1.6 - x0 - 1 + 0.01, ROLL_Y - r1 - 12 + 1 + 0.01, 14.01)


def _enc_rot(joint):
    """M3: the rotating side of the yaw / inline-roll AS5048A (holder + magnet), unioned into the part it turns with."""
    from . import encoder as en
    _, rot = en.yaw_parts() if joint == "yaw" else en.roll_parts()
    out = rot[0][1]
    for _n, q, _c in rot[1:]:
        out = out + q
    return out


def _enc_static():
    """M3: the static AS5048A boards (yaw on the top yaw plate; roll on the spring ring's bridge, inline only)."""
    from . import encoder as en
    out = [("yaw_enc_pcb", en.yaw_pcb(), "green")]
    if INLINE:
        out.append(("roll_enc_pcb", en.roll_pcb(), "green"))
    return out


def module_inline():
    out = [("ferrule_m", Rot(-90, 0, 0) * ferrule(male=True), "orange"),
           ("roll_link", roll_link_inline(), "white"), ("roll_gim_housing", roll_motor_housing(), "dark"),
           ("roll_outflange", roll_outflange(), "steel"), ("roll_arm_rear", roll_arm_rear(), "white"),
           ("roll_spring_hub", roll_spring_hub() + _enc_rot("roll"), "dark"), ("roll_spring_ring", roll_spring_ring(), "dark"),
           ("yaw_block", clevis_and_yaw_block(), "white"), ("yaw_shaft", yaw_shaft() + _enc_rot("yaw"), "orange"), ("yaw_flange", yaw_flange() + yaw_tie_ring(), "steel"),
           ("yaw_plate_bot", yaw_plate_bot(), "white"), ("yaw_spring_ring", yaw_spring_ring(), "dark"), ("yaw_band", yaw_band(), "white")]
    out += [(f"yaw_{n}", p, c) for n, p, c in _stage(YAW_T, 20)]
    out.append(("yaw_stop_pin", yaw_stop_pin(), "steel"))
    if GUARD == "latch":                                                          # R17 power-off roll latch (rnd/power_off.py O2)
        out += latch_parts()
    return out + _enc_static()                                                    # M3: AS5048A boards (rnd/parts/encoder.py)


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
    if INLINE:                                                                    # R2b: 2 × M4 up into the +X end wall (end_wall)
        for s in (-1, 1):
            pl -= Pos((END_WALL["x0"] + X_PL1) / 2, ROLL_Y + s * END_WALL["bolt_y"], P2[0] - 1) * Cylinder(2.25, 10, align=MIN)
    return pl


def clevis_and_yaw_block() -> Part:
    p = None
    for z0, z1 in (P1,):                                                           # D19: P2 is yaw_plate_bot()
        pl = _plate_z(-YAW_C - 48, X_PL1, ROLL_Y - 46, ROLL_Y + 46, z0, z1, 16)
        pl -= Pos(0, ROLL_Y, 0) * Cylinder(26, 400)                                # 61808 seat, yaw shaft
        pl -= YAW_T * wall_a_cuts_v4(None, EAR_SITE)                              # D26: Ø40.6 pulley hole, spigot counterbore, ear bolts + nut pockets
        from .encoder import yaw_pilot_cuts
        pl -= yaw_pilot_cuts()                                                    # M3: 2 × M2 pilots for the yaw AS5048A board
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
            arm -= Pos(0, ROLL_Y + ADAPT["pcd"] / 2 * math.cos(a), ADAPT["pcd"] / 2 * math.sin(a)) * Rot(0, 90, 0) * Cylinder(OUT_CLAMP["sleeve"][0] / 2, 400)   # crush sleeve Ø8, press fit (bearing path)
        ri, ro, t = OUT_CLAMP["ring"]                                                 # steel clamp ring under the 6 low heads
        arm -= Pos(x1 - t - OUT_CLAMP["head_h"], ROLL_Y, 0) * Rot(0, 90, 0) * (Cylinder(ro + 0.2, 20, align=MIN) - Cylinder(ri - 0.2, 20, align=MIN))
        p += arm
        return p + end_wall()
    return _clevis_belt(p)


# R2b (2026-11-27, rnd/stiffness.py): the bottom plate (yaw bearing B) reached the +X clevis arm only through the side
# walls, which the Ø88 spring pocket cuts to 2 mm at x 0: under the yaw-seat couple it tilted 2.2 mrad (P2), 1.4 mm at
# the hand. A wall right under the clevis arm (x 47 … X_PL1, full plate width, plate to plate) closes the box at +X and
# ties the bottom plate straight to the arm: 1.39 → 0.40 mm, + 34 cm³ (≈ 26 g). It belongs to the top half (as the side
# walls); the bottom plate bolts to it with 2 × M4 into heat-set inserts. A 16 × (CAV_H + 3) slot at its foot keeps the
# ribbon's way out of the yaw spring ring (exit at x 41.5 … 45.5) to the clevis arm. Inline roll only (belt: X_PL1 48).
END_WALL = dict(x0=47.0, slot_w=16.0, bolt_y=30.0, insert_d=5.6, insert_l=8.0)


def end_wall() -> Part:
    e = END_WALL
    x0, x1, z0, z1 = e["x0"], X_PL1, P2[1], P1[0]
    w = Pos((x0 + x1) / 2, ROLL_Y, (z0 + z1) / 2) * Box(x1 - x0, 92, z1 - z0)
    w -= Pos((x0 + x1) / 2, ROLL_Y, z0 + (CAV_H + 3) / 2) * Box(x1 - x0 + 1, e["slot_w"], CAV_H + 3)
    for s in (-1, 1):
        w -= Pos((x0 + x1) / 2, ROLL_Y + s * e["bolt_y"], z0) * Cylinder(e["insert_d"] / 2, e["insert_l"], align=MIN)
    return w


def _clevis_belt(p):
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
    h = YAW_TOP - z0
    s = Pos(0, ROLL_Y, z0) * (Cylinder(20, h, align=MIN) - Cylinder(YF["bore_r"], 200, align=MIN))
    s += Pos(0, ROLL_Y, YAW_BELT - 8) * (pulley(40, 16, flange=False) - Cylinder(YF["bore_r"], 16, align=MIN))
    s += Pos(0, ROLL_Y, P2[1]) * (Cylinder(21.6, 1.0, align=MIN) - Cylinder(YF["bore_r"], 1.0, align=MIN))
    s += Pos(0, ROLL_Y, P2[1] + 1.0) * (Cylinder(YAW_SPR[0], YAW_BELT - 8 - P2[1] - 1.0, align=MIN) - Cylinder(YF["bore_r"], 30, align=MIN))
    s -= Pos(0, ROLL_Y, z0) * Rot(0, 0, 30) * _hex(YF["hex_af"] + 0.4, YF["hex_h"] + 0.2)   # torque socket for the flange's hex spigot
    t = YAW_TIE
    top = YAW_TOP
    s -= Pos(0, ROLL_Y, top - t["cb_depth"]) * Cylinder(t["cb_r"], 10, align=MIN)      # C1: counterbore for the clamp ring + heads
    for a in _yaw_tie_angles():                                                   # C1: 6 × Ø3.4 through the wall (no inserts)
        s -= Pos(t["r"] * math.cos(a), ROLL_Y + t["r"] * math.sin(a), z0 - 1) * Cylinder(t["sleeve_hole"] / 2, top - z0 + 2, align=MIN)   # Neon 3: crush sleeve
    return s


# Yaw flange ↔ tube joint (Astra #2b finding 2, D20 payload): a hollow hex spigot on the flange (AF 24 × 8, Ø16 ribbon bore)
# in a hex socket in the tube end carries the torque by form.
# C1 (Fable review 2026-11-27): the bending and hanging load were on 6 × M3 into heat-set inserts in the tube end, in
# pull-out. Now the flange is **6061-T6** (machined, ESTIMATE $25) and the joint is a tie: 6 × M3 × 40 class 12.9 socket
# caps (ISO 4762) go down from the tube's top end, heads on a steel clamp ring (r 8.5–16.8 × 2) in a Ø34 × 5.5 counterbore
# flush under the top, through Ø3.4 holes in the tube wall (bore Ø16 for the ribbon, wall r 8–20) and the hex-socket
# void, into M3 threads tapped in the aluminium spigot + plate (≈ 10.8 mm engaged, Loctite 243). The whole printed tube
# is clamped in compression between the ring and the flange, so the hanging load goes flange → screws → ring → tube
# (compression) → shoulder → bearing B, and the tube's bending at bearing B is prestressed. Screws at the hex corners
# (r 11, 30° + k·60°). Tightened from above before the yaw block goes onto the roll output. Capacity: rnd/joints.py.
YF = dict(hex_af=24.0, hex_h=8.0, bore_r=8.0, spigot_r_in=8.0)
# Neon review 2026-11-27 (finding 3): the 37.7 mm printed tube was the clamped member; 36 µm of screw stretch and
# ≈ 0.14 mm of the print's own squeeze were the springs: ≈ 30 % of the preload left at a creep coefficient of 2, and the
# joint then gaps under the basket wrench (rnd/joints.py). Each screw now runs in a steel crush sleeve (Ø4.6 / Ø3.2,
# turned 42CrMo4+QT) from the clamp ring to the flange's spigot top, so the clamp is ring → sleeve → 6061 flange in
# metal, raised to 1.2 kN (the print no longer limits it); the print is sandwiched with a 0.05 mm squeeze. The sleeve
# leaves 0.65 mm of wall to the Ø16 ribbon bore. Rows: rnd/joints.py.
YAW_TIE = dict(n=6, r=11.0, ang0=30.0, d=3.0, pitch=0.5, As=5.03, hole=3.4, tap=2.5, grade="12.9", proof=970.0, preload=1200.0, preload_bare=1000.0,
               ring=(8.5, 16.8, 2.0), cb_r=17.0, head_h=3.0, top_gap=0.5, L=40.0, tau_flange=207.0, sleeve=(4.6, 3.2), sleeve_hole=4.7, squeeze=0.05,
               bell="M3", n_bell=0)   # M3 (encoder): L 45 → 40 with the tube top 2.5 mm lower
from .shoulder import _stack_h
YAW_TIE["stack_h"] = _stack_h(YAW_TIE["bell"], YAW_TIE["n_bell"], YAW_TIE["preload"])
YAW_TIE["cb_depth"] = YAW_TIE["ring"][2] + YAW_TIE["stack_h"] + YAW_TIE["head_h"] + YAW_TIE["top_gap"]    # 5.5 mm


def _yaw_tie_angles():
    return [math.radians(YAW_TIE["ang0"] + 360.0 * k / YAW_TIE["n"]) for k in range(YAW_TIE["n"])]


def yaw_tie_levels():
    """z of the ring seat, the under-head plane, the screw tip and the thread engaged in the flange (mm)."""
    t = YAW_TIE
    top = YAW_TOP
    seat = top - t["cb_depth"]
    head = seat + t["ring"][2] + t["stack_h"]
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
           ("yaw_block", clevis_and_yaw_block(), "white"), ("roll_clevis_cap_in", roll_clevis_cap(0), "white"), ("roll_clevis_cap_out", roll_clevis_cap(1), "white"), ("yaw_shaft", yaw_shaft() + _enc_rot("yaw"), "orange"), ("yaw_flange", yaw_flange() + yaw_tie_ring(), "steel"),
           ("yaw_plate_bot", yaw_plate_bot(), "white"),
           ("yaw_spring_ring", yaw_spring_ring(), "dark")]
    out += [("shoulder_cap", shoulder_cap(), "white"), ("yaw_band", yaw_band(), "white")]
    out += [(f"roll_{n}", p, c) for n, p, c in _stage(ROLL_T, 18)]
    out += [(f"yaw_{n}", p, c) for n, p, c in _stage(YAW_T, 20)]
    out.append(("yaw_stop_pin", yaw_stop_pin(), "steel"))
    return out + _enc_static()                                                    # M3: AS5048A boards (rnd/parts/encoder.py)


def _rear_metal_mass():
    """kg: the rear plate (7075) + sleeves and stop bushes (steel), from the CAD volumes."""
    x0, x1 = X_PLATE
    plate = math.pi * (PLATE_R[1] ** 2 - PLATE_R[0] ** 2) * (x1 - x0) * 2.81e-6
    return round(plate + (rear_metal().volume - plate / 2.81e-6) * 7.85e-6, 4)


def masses(density=1.27, split=False):
    """CAD roll-up (loop 40). Printed parts at 60–80 % fill; D13 standard parts by material (steel shaft, aluminium
    pulley, 6801 at 60 % of solid steel). split=True → (roll-body mass, yaw-stage mass) for params.
    D24 (2026-11-27): each stage is small_stage_v3 + the bolted cartridge; as in shoulder.masses the sleeve counts only by
    what it adds over the loop-10 can (nose, flange, rear cap), the Oldham drive replaces the KX74 hub + collar + coupling."""
    from .belt_stage import small_stage_v4
    st = dict((n, p) for n, p, _ in small_stage_v4(20))
    d13 = ((st["shaft_12"].volume + st["kx_hub_v2"].volume) * 7.85e-3 + st["al_pulley"].volume * 2.7e-3
           + st["brg_B_6001"].volume * 7.85e-3 * 0.6 + st["brg_B_bushing"].volume * density * 0.8e-3) / 1e3   # D26 direct stage, one, kg
    sock = ((cm.can_bolted().volume + cm.can_cap_bolted().volume - cm.can().volume) * density * 0.8e-3) / 1e3 + 0.006   # + ear screws, inserts (no Oldham, D26)
    pr = lambda v, f: v * f / 1e6 * density
    if INLINE:      # the GIM8108-36 itself (1.0 kg) is the roll actuator mass in params.arm.joints, not counted here
        roll = {"roll link: ferrule boss, frame, motor floor, support stub, spring bar (printed)": pr(roll_link().volume, 0.6),
                "roll_arm_rear (printed)": pr(roll_arm_rear().volume, 0.6),
                "roll spring hub + ring (printed)": pr(roll_spring_hub().volume + roll_spring_ring().volume, 0.8),
                "Neon 6b/6d: 7075 rear plate + 6 steel sleeves + 2 × 42CrMo4 stop bushes (CAD)": _rear_metal_mass(),
                "61806 + 12 × M4 + 2 × M5 + DIN 471 ring (ESTIMATE)": 0.045,
                "male ferrule (printed) + dowels + screws + ribbon (ESTIMATE)": pr(67213, 0.7) + 0.04}
        if GUARD == "latch":
            roll["R17 roll latch: pin + sleeve + spring + cable + solenoid (ESTIMATE; not in params.ROLL_DRIVES)"] = LATCH["mass"]
    else:
        roll = {"roll link + seat caps + spring ring/bar (printed)": pr(roll_link().volume + roll_seat_cap(W1).volume + roll_seat_cap(W2).volume, 0.6) + pr(roll_spring_ring().volume, 0.8),
                "roll shaft + 80T (one piece, D19) + spring hub": pr(roll_shaft().volume, 0.7) + pr(roll_spring_hub().volume, 0.8),
                "roll D26 direct stage + bolted can flange/nose/cap": d13 + sock, "2 × 61808 (ESTIMATE)": 0.04, "male ferrule (printed) + dowels + belt + screws + ribbon (ESTIMATE)": pr(67213, 0.7) + 0.05}
    caps = 0.0 if INLINE else roll_clevis_cap(0).volume + roll_clevis_cap(1).volume
    yaw = {"clevis + caps + yaw block plates + posts (printed)": pr(clevis_and_yaw_block().volume + caps + yaw_plate_bot().volume, 0.6),
           "yaw shaft + 40T + spring hub (one piece), spring ring (printed)": pr(yaw_shaft().volume, 0.7) + pr(yaw_spring_ring().volume, 0.8),
           "C1 yaw flange (6061-T6) + steel clamp ring + 6 × M3 × 40 12.9 + 6 crush sleeves": yaw_flange().volume * 2.7e-6 + yaw_tie_ring().volume * 7.85e-6
           + 6 * (5.03 * YAW_TIE["L"] + 45) * 7.85e-6 + YAW_TIE["n"] * math.pi / 4 * (YAW_TIE["sleeve"][0] ** 2 - YAW_TIE["sleeve"][1] ** 2) * 30.0 * 7.85e-6,
           "yaw D26 direct stage + bolted can flange/nose/cap": d13 + sock, "2 × 61808 + stop pin + belt + screws + ribbon (ESTIMATE)": 0.075}
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
        # that 3 mm of M4 in aluminium can't hold. Neon review (finding 6a): bearing allowable back to 150 MPa (unconfirmed
        # cast boss) and the first dowel to touch carries more (rnd/joints.py roll_rows, same numbers)
        from .. import joints as _j
        row = [r for r in _j.roll_rows() if r["item"].startswith("M1 output boss")][0]
        cap = row["margin"] * m["peak"]
        basis0 = row["basis"]
        rows.append(("roll (inline)", "motor output ↔ 7075 adapter (dowels, load sharing, + 3 × M4 friction) vs the drive limit (35.1)", cap, m["rated"], basis0))
        rows.append(("roll (inline)", "motor output ↔ 7075 adapter (dowels, load sharing, + 3 × M4 friction) vs stall", cap, m["peak"], basis0 + f"; stall {m['peak']:.1f} N·m (actuators.yaml)"))
        # (b) metal adapter → printed clevis arm (Neon review: crush sleeves, see OUT_CLAMP)
        from .. import joints as _j
        L = o["arm_t"] - o["ring"][2] - o["head_h"]
        A_sl = math.pi / 4 * (o["sleeve"][0] ** 2 - o["sleeve"][1] ** 2)
        c_b, c_s = (L + o["ring"][2] + 3.0) / (_j.E_STEEL * 14.2), L / (_j.E_STEEL * A_sl)
        c_p = L / (_j.E_PRINT * math.pi * (o["ring"][1] ** 2 - o["ring"][0] ** 2) / A["n"])
        F_bare = _j.life_preload(o["preload"], c_b, c_p, L_p=L, n_print_if=2)
        F_life = _j.life_preload(o["preload"], c_b + c_s, 0.0, n_metal_if=4, phi=0.0)
        t_fr = o["mu_metal"] * A["n"] * F_life * r1                                     # ring ↔ sleeve ↔ adapter, steel on 7075
        t_br = A["n"] * o["sleeve"][0] * L * 12.0 * r1                                   # sleeves bearing in the print at the 12 MPa across-layer allowable
        basis = (f"{A['n']} × M5 on PCD {A['pcd']:.0f} in Ø{o['sleeve'][0]:.0f}/{o['sleeve'][1]} crush sleeves, {o['preload']:.0f} N → {F_life:.0f} N after embedding (metal stack; "
                 f"wave 3 clamped {L:.1f} mm of PETG: {F_bare:.0f} N left at φ {_j.CREEP['phi']:.0f}, friction {o['mu'] * A['n'] * F_bare * r1:.0f} N·m); friction μ {o['mu_metal']} "
                 f"steel/7075 (ESTIMATE) adapter → sleeves; the print takes the torque in bearing on the press-fitted sleeves ({t_br:.0f} N·m at 12 MPa, no slip clearance)")
        basis += f"; the friction ({t_fr:.0f} N·m: a {L:.1f} mm metal stack loses most of its preload to embedding) is not counted"
        rows.append(("roll (inline)", "adapter ↔ clevis arm: sleeves bearing in the print vs the drive limit (35.1)", t_br, m["rated"], basis))
        rows.append(("roll (inline)", "adapter ↔ clevis arm: sleeves bearing in the print vs stall", t_br, m["peak"],
                     f"{A['n']} × Ø{o['sleeve'][0]:.0f} × {L:.1f} mm at 12 MPa (fea.py across-layer allowable) = {t_br:.0f} N·m; stall only on a fault"))
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
    from .. import joints
    J = joints.joint_models()["yaw flange ↔ tube"]
    t_cl = t["n"] * J["F_life"] * 0.30 * t["r"] / 1000
    rows.append(("yaw", "tube ↔ flange, hex spigot AF 24 × 8 (form)", min(t_hex, t_root), T_yaw,
                 f"flats {t_hex:.0f} N·m (3 mm strips at 25 MPa in-layer), spigot root {t_root:.0f} N·m (torsion at 12 MPa across layers); "
                 f"+ {t_cl:.0f} N·m clamp friction from the YAW_TIE (μ 0.3) not counted; demand = GIM6010-8 peak × 2 × 0.95"))
    try:
        import json as _json
        dem = _json.loads((joints.DOCS / "joints.json").read_text())
        dem = max(r["M"] for r in dem["joints"] if r["joint"] == "yaw flange ↔ tube" and r["case"] in joints.SERVICE)
        src = "rnd/joints.py: the twin's worst basket/hug wrench × 1.5"
    except (OSError, ValueError, KeyError):
        dem, src = 20.7 * 1.5, "20.7 N·m × 1.5, rnd/basket.py worst shoulder-pitch torque (run python -m rnd.joints for the twin value)"
    mg = min(m.m_gap(0.0) for m in J["models"])
    rows.append(("yaw", "tube ↔ flange, C1 tie (6 × M3 12.9 into the 6061 flange): moment before the joint gaps vs the service bending", mg, dem,
                 f"clamp model (rnd/joints.py): {t['n']} × {J['F_life'] / 1e3:.2f} kN life preload in steel crush sleeves (metal stack, the print sandwiched); demand {src}; "
                 "fault cases (140 N bump, stalls) in docs/rnd/joints.md"))
    return [dict(joint=j, path=pth, cap=round(c, 1), demand=round(dm, 1), ratio=round(c / dm, 2), basis=b) for j, pth, c, dm, b in rows]
