"""Cartridge M: a GIM6010-8 (or a DM-J4310 with its own adapter) in a printed can.

Load path (KX v1): torque only. Output side: the KX74 male face on a motor adapter. Housing side:
a Ø6 steel torque pin that drops into a radial slot in the bone (precedent: shaft-mounted gearbox torque arm).
Only `motor_adapter` and the can's bolt circle depend on the motor; numbers marked ESTIMATE are
confirmed with `coupon_motor` (radial slots) when the first motor arrives.
"""
from __future__ import annotations

import math

from build123d import Align, Box, Cylinder, Part, Pos, Rot, Axis, SortBy, fillet

from .kx import KX74, male, balls

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
MOTOR = dict(od=80.0, h=40.0,                     # with SDC104 driver (manual: Ø80 × 40)
             out_holes=8, out_pcd=34.0,           # 8×M3 on the output (count from the drawing; PCD ESTIMATE)
             rear_holes=6, rear_pcd=57.0,         # 6×M3 on the housing (count from the drawing; PCD ESTIMATE)
             oldham=50.0, nose=(26.5, 29.5), flange=(68.0, 29.0))   # D24 bolted can: Oldham OD, nose bore / outer r, flange disc OD / v1 bolt r (v2 ears: ear_r())
# Fable M5 (2026-11-27): the D40 acetal Oldham disc is catalogued at ≈ 9–10 N·m, under the GIM6010's 11 N·m peak; D50
# (≈ 14.8 N·m scaled by R², ESTIMATE) → 1.35× (rnd/belts.py shaft_joints). D20 uses the D63 below.
# D20 (2026-11-27, behind params.ARM_MOTORS = "d20"): GIM8108-8 at shoulder pitch + elbow. No drawing yet (reseller data
# only): Ø96 frame ESTIMATE (rnd/specs/actuators.yaml; SteadyWin's GIM8108-9 installation drawing lists Ø96 / Ø84 / Ø70 /
# Ø56 / Ø30 circles), height 50 with the driver ESTIMATE, output 8 × M3 on PCD 30 ESTIMATE, housing 6 × M3 on PCD 70
# ESTIMATE. The 22 N·m stall (2× the GIM6010's 11) doubles the Oldham tongue load, so a Ø50 Oldham (tongue bearing
# ≈ 3T/(2·h·R²) ≈ 17 MPa at 22 N·m vs 26 MPa on the Ø40; hand calc, ESTIMATE) in an Ø84 flange disc; D24 v2 (Fable M6):
# D63 Oldham (nose r 32.5 / 35.5), ears at r 56 outside the Ø102.6 sleeve (22 / (4 × 56 mm) ≈ 98 N per screw), the shared
# Ø12 stage with a steel keyed 20T (belt_stage.AL_PULLEY note) and an 18 N·m flash torque limit; parts_list(MOTOR_8108).
MOTOR_8108 = dict(od=96.0, h=50.0, out_holes=8, out_pcd=30.0, rear_holes=6, rear_pcd=70.0,
                  oldham=63.0, nose=(32.5, 35.5), flange=(84.0, 37.0))   # D63: the 22 N·m stall (D50 ≈ 14.8 N·m would be 0.67×)
MOTORS = {"GIM6010-8": MOTOR, "GIM8108-8": MOTOR_8108}
WALL, FLOOR = 3.0, 4.0
NECK, NECK_R = 5.0, 33.0      # spacer under the male flange: room for the collar teeth behind the lugs
TORQUE_PIN = dict(d=6.0, r=30.0, stick_out=10.0, ang=0.0)


def can() -> Part:
    od = MOTOR["od"] + 2 * WALL + 0.6
    h = MOTOR["h"] + FLOOR
    body = Cylinder(od / 2, h, align=MIN)
    body -= Pos(0, 0, FLOOR) * Cylinder(MOTOR["od"] / 2 + 0.3, h, align=MIN)
    body = fillet(body.edges().group_by(Axis.Z)[-1].sort_by(SortBy.RADIUS)[-1], 1.2)
    body = fillet(body.edges().group_by(Axis.Z)[0].sort_by(SortBy.RADIUS)[-1], 3.0)
    for k in range(0, MOTOR["rear_holes"], 2):            # loop 10 (R10): 3 of the 6 housing holes are enough (torque pin carries the torque)
        a = 2 * math.pi * k / MOTOR["rear_holes"]
        body -= Pos(MOTOR["rear_pcd"] / 2 * math.cos(a), MOTOR["rear_pcd"] / 2 * math.sin(a), 0) * Cylinder(1.7, 20)
        body -= Pos(MOTOR["rear_pcd"] / 2 * math.cos(a), MOTOR["rear_pcd"] / 2 * math.sin(a), 0) * Cylinder(3.1, 3.0)
    body -= Cylinder(13, 20)                                              # rear vent / cable pass-through
    body -= Pos(od / 2, 0, h / 2 + 4) * Box(12, 16, h - 8)                # side window: CAN + power pigtail
    body -= Pos(0, 0, h - 6) * (Cylinder(od / 2 + 1, 2) - Cylinder(od / 2 - 1.2, 2))   # light-ring groove
    tp = TORQUE_PIN
    body -= Pos(tp["r"], 0, 0) * Cylinder(tp["d"] / 2 - 0.05, 2 * FLOOR)  # press-fit seat for the steel torque pin
    return body


def torque_hanger(reach: float, width: float = 48.0, t: float = 7.0, flange: float = 26.0, slot_in: float = 6.0,
                  slot_out: float = 10.0) -> Part:
    """Standard torque-reaction part (confidence campaign): a printed plate with an open radial fork slot for the Ø6
    torque pin, hanging from a flange that screws to the nearest flat structure (2 × M4 into heat-set inserts).
    Local frame: pin axis +Z through the origin, radial direction +X (away from the cartridge axis), plate z ∈ [2, 2 + t],
    mount face at x = reach. The fork is open at the inner end (−X), so the cartridge keeps its radial float.
    Load: GIM peak 11 N·m / r 30 = 367 N along ±Y."""
    tp = TORQUE_PIN
    x0 = -slot_in - tp["d"] / 2
    p = Pos((x0 + reach) / 2, 0, 2) * Box(reach - x0, width, t, align=MIN)
    p += Pos(reach - 3, 0, 2 + t - flange) * Box(6, width, flange, align=MIN)                     # flange against the mount face
    for y in (-width / 2 + 8, width / 2 - 8):
        p -= Pos(reach - 3, y, 2 + t - flange / 2 - 3) * Rot(0, 90, 0) * Cylinder(2.2, 8)         # M4 clearance
    p -= Pos((x0 + slot_out) / 2 - 1, 0, 2) * Box(slot_out - x0 + 2, tp["d"] + 0.4, t, align=MIN)  # open fork slot
    return p


def torque_pin() -> Part:
    tp = TORQUE_PIN
    return Pos(tp["r"], 0, FLOOR - 0.5 - tp["stick_out"] - FLOOR + 0.5) * Cylinder(tp["d"] / 2, tp["stick_out"] + FLOOR - 0.5, align=MIN)


def motor_adapter() -> Part:
    """Bolts to the motor output (8×M3). Male face on top, mating face up (+Z) at z = NECK + face_t."""
    p = Cylinder(NECK_R, NECK, align=MIN) + Pos(0, 0, NECK + KX74.face_t) * Rot(180, 0, 0) * male(KX74, bore=8)
    for k in range(0, MOTOR["out_holes"], 2):             # loop 10 (R10): 4 of 8 output screws (11 N·m peak → 160 N shear per M3)
        a = 2 * math.pi * k / MOTOR["out_holes"] + math.pi / 8
        x, y = MOTOR["out_pcd"] / 2 * math.cos(a), MOTOR["out_pcd"] / 2 * math.sin(a)
        p -= Pos(x, y, 0) * Cylinder(1.7, 40)
        p -= Pos(x, y, NECK + KX74.face_t - 3.5) * Cylinder(3.1, 10, align=MIN)   # counterbores
    return p


def adapter_balls() -> Part:
    return Pos(0, 0, NECK + KX74.face_t) * Rot(180, 0, 0) * balls(KX74)


def motor_dummy() -> Part:
    return Pos(0, 0, FLOOR) * Cylinder(MOTOR["od"] / 2, MOTOR["h"], align=MIN)


def stack_height() -> float:
    """Can floor to the KX mating plane."""
    return FLOOR + MOTOR["h"] + NECK + KX74.face_t


def coupon_motor() -> Part:
    """3 mm slotted disc: bolt it to the motor's front and back faces to read the real bolt circles."""
    p = Cylinder(MOTOR["od"] / 2, 3, align=MIN) - Cylinder(5, 3, align=MIN)
    for k in range(6):                                   # housing pattern: slots spanning PCD 48–66
        p -= Rot(0, 0, 60 * k) * Pos(28.5, 0, 0) * Box(9 + 3.4, 3.4, 6)
    for k in range(8):                                   # output pattern: slots spanning PCD 26–42
        p -= Rot(0, 0, 45 * k + 22.5) * Pos(17, 0, 0) * Box(8 + 3.4, 3.4, 6)
    for k, r in enumerate(range(13, 42, 2)):             # engraved ruler: one tick per 2 mm of radius
        p -= Rot(0, 0, 30) * Pos(r, 0, 2.6) * Box(0.6, 3 if k % 5 else 5, 1)
    return p


# ---------------------------------------------------------------------------------------------------------------------
# D24 (owner, 2026-11-27): bolted cartridge M, no KX socket. Used by every GIM6010 belt stage (shoulder pitch, elbow, yaw,
# belt roll, lift) through belt_stage.bolted_stage / small_stage_v3 / wall_a_cuts.
# D24 v2 (2026-11-27, Fable review C3). v1 (6 × M4 countersunk from the pulley side into inserts in a Ø68 flange, a plain
# 4 mm rear cap "held by 3 radial M3") could not react the motor torque as drawn and could not be swapped:
#   - the cap had no holes and a 4 mm rim cannot hold an M3 insert, so the motor reaction had no path to the sleeve;
#   - the flange inserts could only go in from the mating face, so the screws pulled them out of their bosses;
#   - a Ø6 driver on the screw axes was blocked by the far wall at pitch and elbow (and the ±75° pair sat under the
#     elbow cassette's M5 spacers).
# v2:
#   - **Flange ears outside the sleeve, screws from the cartridge side.** 2–4 ears at radius ear_r() = sleeve r + 3 (a Ø6
#     driver shank) + 1.7 clearance (GIM6010: r 48; GIM8108-8: r 56), so a straight driver runs past the motor sleeve with
#     the belt, pulley and far wall untouched. Each ear takes an ISO 4762 M4 × 12 with an ISO 7089 washer under the head,
#     through the ear (6 mm) and wall A (6 mm) into an ISO 4032 M4 nut in a hex pocket on wall A's pulley-side face: a
#     through-bolt, every printed layer in compression (no insert in pull-out anywhere). Nut pocket 3.4 deep → 2.6 mm floor.
#   - Preload 0.75 kN at 0.6 N·m (F = T / (K·d), K 0.2, ESTIMATE): 16 MPa under the washer, 28 MPa under the nut
#     (26.5 mm² hex bearing) vs ≈ 50 MPa PETG-CF compressive (ESTIMATE). Friction capacity μ 0.3 × 4 × 0.75 kN × r 48 =
#     43 N·m vs the 11 N·m GIM6010 peak (3.9×); GIM8108-8: × r 56 = 50 N·m vs 22 N·m (2.3×), before bolt shear.
#     μ 0.3 PETG on PETG is an ESTIMATE (coupon C-series).
#   - Ear angles per site (EAR_SITES, from the stage +X = the belt direction), each set by the R14 (c) driver probe on
#     the whole robot: elbow 4 ears (±40°/±140°), pitch 3 (40°, −30°, −70°), yaw 2 (±145°), lift 2 (±140°). With 2 ears
#     the friction capacity is 2 × 0.3 × 0.75 kN × r 48 = 22 N·m vs 11 N·m (2×); the spigot takes the radial load.
#   - **Rear cap v2:** a 4 mm plug in the sleeve bore carries the motor (3 × M3 into the housing pattern, low heads in
#     counterbores) and has two 3 mm ears that sit flush in notches in the sleeve's rear end (torque dogs) at ±90°. The
#     notches run into two lugs on the sleeve (r → sleeve + 6.2, 10 long, so at the elbow they stay outside the −Y wall);
#     an M3 × 10 goes through each cap ear into a short heat-set insert (Ø4.0 × 4.0, ESTIMATE size) pressed into the lug
#     from its FRONT face, so the screw pushes the insert into its boss (3 mm of lug between the insert and the notch).
#     The cap screws clamp the motor's front face onto the sleeve step: friction μ 0.3 × ≈ 1 kN × r 31 ≈ 9 N·m plus the two
#     dogs (11 N·m / (2 × 46 mm) ≈ 120 N on 3 × 3 mm flanks ≈ 13 MPa at the GIM6010 peak; ESTIMATE).
#     Assumed: the GIM6010-8's 6 × M3 housing holes are on the driver (rear) face (manual ESTIMATE, Fable C3 (S)). If the
#     seller's drawing puts them on the output face, the 3 mm step takes them instead (screws from the 16 mm gap between
#     the flange and the step) and the cap keeps only its retention role. Read the real faces with coupon_motor().
#   - Drive: an Oldham coupling (blind axial mate, absorbs the ±1 mm the eccentric bushings move the stage shaft):
#     aluminium clamp hub with a 4 mm keyway on the Ø12 stage shaft (off the shelf, D50; Fable M5: D40 → D50) → acetal
#     disc → printed ASA tongue adapter on the motor output (4 × M3, PCD 34). Tongue bearing ≈ 3T/(2·h·R²) ≈ 6.6 MPa at the
#     11 N·m peak (hand calc; coupon).
#   - Swap (rnd/assembly_seq.py, R14 (c)/(d)): 4 ear screws from the cartridge side, the cartridge slides out along its
#     axis off the Oldham disc; belt, pulley, shaft, bearings and the shaft-side Oldham hub stay.
# Stage frame (as small_stage_v3): flange mating plane z = 0 (= the stage wall's cartridge-side face), cartridge +Z.
# All dimensions not on a datasheet are ESTIMATE.
OLDHAM = dict(od=50.0, hub_l=15.0, tongue_h=4.0, tongue_w=10.0, disc_t=10.0, slot_d=4.0, gap=0.3)   # D50 clamp-hub Oldham, 12 mm bore + 4 mm keyway (ESTIMATE; od per motor: MOTOR["oldham"])
ADAPTER_T = 4.0
NOSE_B = dict(r_in=21.5, r_out=24.5, lip=1.5)
FLANGE_B = dict(od=68.0, t=6.0, bolt_r=29.0, insert_d=5.6, insert_l=5.7, clr_d=4.6, csk_d=8.6)   # D24 v1 (legacy: only roll.flange_screw_access still reads it)
FLANGE_ANG = (75.0, -75.0, 115.0, -115.0, 155.0, -155.0)                                         # D24 v1 (legacy, as above)
# D24 v2 ears. Screw ISO 4762 M4 × 12 + ISO 7089 washer (Ø9 × 0.8), nut ISO 4032 M4 (s 7, m 3.2): ISO catalogue sizes.
EAR = dict(t=6.0, boss_r=5.5, clr_d=4.5, head_d=7.0, head_h=4.0, washer=(9.0, 0.8), nut_af=7.0, nut_m=3.2, pocket=3.4,
           driver_r=3.0, driver_clear=1.7, screw="ISO 4762 M4 × 12 + ISO 7089 washer, ISO 4032 M4 nut in a hex pocket")
# Ear angles per site (deg from the stage +X), chosen with rnd/assembly_seq.py's driver probe on the whole robot. A module
# sets EAR_SITE = "<site>" and passes site=EAR_SITE to bolted_stage / bolted_cartridge / wall_a_cuts; the probe reads it.
EAR_SITES = {
    "elbow": (40.0, -40.0, 140.0, -140.0),   # clear of the cassette posts (stage (10, ±35)), the elbow tube, the −Y seat boss, the housing sides
    "pitch": (40.0, -30.0, -70.0),           # the lift carriage sits 72 mm behind the rear (±140°) pair and the balancer cage over 50–130°; +40° by a ball-end past the AI bay
    "yaw": (145.0, -145.0),                  # the −X clevis arm's foot covers |a| ≤ 40°, the block posts (stage (40, ±38), (−24, ±38)) and side walls the rest
    "lift": (140.0, -140.0),                 # the lift can sits 1.2 mm over the deck and its cover wraps it 3 mm off: only the lower diagonals
    "roll_belt": (40.0, -40.0),              # belt-roll fallback: the walls end 37 mm over the stage axis on the −Z side (unchecked)
}
EAR_SITES["default"] = EAR_SITES["yaw"]      # the fallback for a caller that names no site: 2 ears, clash-free at every stage
# Side exit (R14 (d)): where the cartridge cannot be pulled its own length along its axis (pitch: the lift carriage is
# 12.6 mm behind the rear; lift: the base rim 13 mm behind), the flange, lip and nose are open on one side (a U-slot as
# wide as the Oldham disc, up to just under the motor-side adapter). The cartridge then lifts 4.5 mm (adapter tongue out
# of the disc, lip out of the counterbore) and slides off sideways AWAY from the opening, so the shaft-side hub and the
# disc (which stays on it) leave through the slot. Opening angle from the stage +X (pitch: +Y = up, the cartridge drops
# out downward; lift: −X = down, it comes out upward); the ears keep ≥ 40° clear of it.
EXIT_SITES = {"pitch": 90.0, "lift": 180.0}
EXIT_LIFT = 4.5                              # axial travel before the side move: adapter tongue (3.7 in the disc) + 0.8
STEP_T = 3.0
CAP_T = FLOOR
# rear cap v2: ears/lugs at ±90° (clear of the pigtail window at 0°, of the deck under the lift can, and 50° from the ears)
CAP = dict(ear_t=3.0, ear_w=10.0, lug_h=10.0, lug_out=6.2, ins_d=4.0, ins_l=4.0, clr_d=3.4, ang=(90.0, -90.0),
           screw="ISO 7380 M3 × 10 into a short M3 heat-set insert (Ø4.0 hole × 4.0)", motor_screw="DIN 7984 M3 × 8 (low head)", cb=(6.2, 2.0))


def bolted_levels(m: dict | None = None) -> dict:
    """Stage-frame z levels of the Oldham drive and the sleeve (flange plane z = 0) for motor m (default the GIM6010-8)."""
    m = m or MOTOR
    o = OLDHAM
    hub = (-0.5, -0.5 + o["hub_l"])                         # clamp hub, tongue in its top tongue_h
    disc = (hub[1] + o["gap"] - o["tongue_h"], hub[1] + o["gap"] - o["tongue_h"] + o["disc_t"])
    ad = (disc[1] + o["gap"], disc[1] + o["gap"] + ADAPTER_T)  # adapter base; its tongue hangs into the disc's upper slot
    face = ad[1]                                            # motor output face
    return dict(hub=hub, disc=disc, adapter=ad, motor_face=face, step=(face - STEP_T, face), flange=(0.0, EAR["t"]),
                rear=face + m["h"] + CAP_T)


def stack_height_bolted(m: dict | None = None) -> float:
    """Can rear face → flange mating plane (the bolted analogue of stack_height())."""
    return bolted_levels(m)["rear"]


def can_r_out(m: dict | None = None) -> float:
    """Outer radius of the bolted can's motor sleeve."""
    return (m or MOTOR)["od"] / 2 + WALL + 0.3


def cap_screw_r(m: dict | None = None) -> float:
    """Rear-cap screw radius: the lug centre (2.3 mm of sleeve to the motor bore, 3.1 mm to the lug's outer edge)."""
    return can_r_out(m) + CAP["lug_out"] - 5.0


def ear_r(m: dict | None = None) -> float:
    """Ear bolt radius: a Ø6 driver shank runs past the sleeve with driver_clear to spare (GIM6010 48.0, GIM8108-8 56.0)."""
    return can_r_out(m) + EAR["driver_r"] + EAR["driver_clear"]


def nose_flange(m: dict | None = None, site: str = "default"):
    """(nose, flange) dicts for motor m: the nose sizes, the flange disc and the v2 ear pattern (bolt_r, ang)."""
    m = m or MOTOR
    return (dict(NOSE_B, r_in=m["nose"][0], r_out=m["nose"][1]),
            dict(od=m["flange"][0], t=EAR["t"], bolt_r=ear_r(m), ang=EAR_SITES[site], clr_d=EAR["clr_d"]))


def ear_fasteners(m: dict | None = None, site: str = "default"):
    """[(name, head-top point, driver axis)] of the flange ear screws in the stage frame (axis +Z: out on the cartridge side)."""
    _, f = nose_flange(m, site)
    z = f["t"] + EAR["washer"][1] + EAR["head_h"]
    return [(f"ear screw {a:+.0f}°", (f["bolt_r"] * math.cos(math.radians(a)), f["bolt_r"] * math.sin(math.radians(a)), z), (0.0, 0.0, 1.0))
            for a in f["ang"]]


def _lobe(r: float, a_deg: float, rb: float, t: float, z0: float = 0.0) -> Part:
    """Ear lobe in z0 … z0 + t: a radial bar (2·rb wide) from the axis to r plus a disc of radius rb at r, at angle a."""
    a = math.radians(a_deg)
    bar = Pos(r / 2 * math.cos(a), r / 2 * math.sin(a), z0) * Rot(0, 0, a_deg) * Box(r, 2 * rb, t, align=MIN)
    return bar + Pos(r * math.cos(a), r * math.sin(a), z0) * Cylinder(rb, t, align=MIN)


def can_bolted(m: dict | None = None, site: str = "default") -> Part:
    """D24 v2 sleeve in the STAGE frame (flange plane z = 0, motor toward +Z): nose lip, flange disc + ears, nose, step,
    motor sleeve with the rear-cap lugs and dog notches."""
    m = m or MOTOR
    L = bolted_levels(m)
    n, f = nose_flange(m, site)
    c = CAP
    r_in, r_out = m["od"] / 2 + 0.3, can_r_out(m)
    z_rear = L["rear"]
    body = Pos(0, 0, L["step"][0]) * Cylinder(r_out, z_rear - L["step"][0], align=MIN)
    body = fillet(body.edges().group_by(Axis.Z)[0].sort_by(SortBy.RADIUS)[-1], 2.0)
    for a in c["ang"]:                                                                    # rear-cap lugs
        body += _lobe(r_out + c["lug_out"] - 5.0, a, 5.0, c["lug_h"], z_rear - c["lug_h"])
    body -= Pos(0, 0, L["motor_face"]) * Cylinder(r_in, z_rear, align=MIN)              # motor bore, open at the rear
    body += Pos(0, 0, f["t"]) * Cylinder(n["r_out"], L["step"][0] - f["t"] + 0.5, align=MIN)   # nose
    fl = Cylinder(f["od"] / 2, f["t"], align=MIN)                                         # flange disc + ears
    for a in f["ang"]:
        fl += _lobe(f["bolt_r"], a, EAR["boss_r"], f["t"])
    body += fl
    body += Pos(0, 0, -n["lip"]) * Cylinder(n["r_out"], n["lip"], align=MIN)             # centring lip (spigot)
    body -= Pos(0, 0, -5) * Cylinder(n["r_in"], L["motor_face"] + 5, align=MIN)          # coupling bore through flange, nose, step
    for a in f["ang"]:
        t = math.radians(a)
        body -= Pos(f["bolt_r"] * math.cos(t), f["bolt_r"] * math.sin(t), -1) * Cylinder(EAR["clr_d"] / 2, f["t"] + 2, align=MIN)
    if site in EXIT_SITES:                                                               # side exit: U-slot through lip, flange, nose
        w = m["oldham"] + 1.2
        body -= Rot(0, 0, EXIT_SITES[site]) * Pos(0, 0, -n["lip"] - 1) * Box(r_out + 20, w, L["adapter"][0] + n["lip"] + 1 + 0.5,
                                                                                 align=(Align.MIN, Align.CENTER, Align.MIN))
    body -= Pos(r_out, 0, (L["motor_face"] + z_rear) / 2) * Box(12, 16, m["h"] - 8)   # side window: CAN + power pigtail
    body -= Pos(0, 0, z_rear - c["lug_h"] - 4) * (Cylinder(r_out + 1, 2) - Cylinder(r_out - 1.2, 2))   # light-ring groove (in front of the lugs)
    r_s = cap_screw_r(m)
    for a in c["ang"]:
        t = math.radians(a)
        body -= Pos(0, 0, z_rear - c["ear_t"]) * Rot(0, 0, a) * Pos((r_in + r_out + c["lug_out"]) / 2, 0, 0) * Box(r_out + c["lug_out"] - r_in + 2, c["ear_w"] + 0.4, c["ear_t"] + 1, align=MIN)   # notch = torque dog
        at = Pos(r_s * math.cos(t), r_s * math.sin(t), 0)
        body -= at * Pos(0, 0, z_rear - c["lug_h"] - 0.01) * Cylinder(c["ins_d"] / 2 - 0.1, c["ins_l"], align=MIN)   # insert, pressed in from the lug's front face
        body -= at * Pos(0, 0, z_rear - c["lug_h"]) * Cylinder(c["clr_d"] / 2, c["lug_h"] + 1, align=MIN)
    return body


def can_cap_bolted(m: dict | None = None) -> Part:
    """D24 v2 rear cap (stage frame): 4 mm plug in the sleeve bore carrying the motor (3 × M3 on its rear PCD, low heads
    counterbored), two 3 mm ears in the sleeve's rear notches (torque dogs), screwed through into the lugs' inserts."""
    m = m or MOTOR
    L = bolted_levels(m)
    z0 = L["rear"] - CAP_T
    c = CAP
    r_out = can_r_out(m)
    r_s = cap_screw_r(m)
    cap = Pos(0, 0, z0) * Cylinder(m["od"] / 2 + 0.2, CAP_T, align=MIN)
    for a in c["ang"]:
        t = math.radians(a)
        cap += Pos(0, 0, L["rear"] - c["ear_t"]) * Rot(0, 0, a) * Pos((r_out + c["lug_out"]) / 2, 0, 0) * Box(r_out + c["lug_out"], c["ear_w"], c["ear_t"], align=MIN)
        cap -= Pos(r_s * math.cos(t), r_s * math.sin(t), z0 - 1) * Cylinder(c["clr_d"] / 2, 10, align=MIN)
    for k in range(0, m["rear_holes"], 2):
        a = 2 * math.pi * k / m["rear_holes"]
        x, y = m["rear_pcd"] / 2 * math.cos(a), m["rear_pcd"] / 2 * math.sin(a)
        cap -= Pos(x, y, z0 - 1) * Cylinder(1.7, 20, align=MIN)
        cap -= Pos(x, y, L["rear"] - c["cb"][1]) * Cylinder(c["cb"][0] / 2, 5, align=MIN)
    cap -= Pos(0, 0, z0 - 1) * Cylinder(13, 20, align=MIN)                               # rear vent / cable pass-through
    return cap


def can_envelope(m: dict | None = None, site: str = "default", clr: float = 0.6, z0: float = 0.0, z1: float = 120.0) -> Part:
    """What the cartridge (without its rear lugs) sweeps when it slides out along +Z: sleeve, flange disc and ears grown by
    clr, extruded z0 … z1 (stage frame). Housings cut it where the cartridge has to pass a wall (the elbow's −Y wall)."""
    m = m or MOTOR
    _, f = nose_flange(m, site)
    h = z1 - z0
    e = Pos(0, 0, z0) * Cylinder(max(can_r_out(m), f["od"] / 2) + clr, h, align=MIN)
    for a in f["ang"]:
        e += _lobe(f["bolt_r"], a, EAR["boss_r"] + clr, h, z0)
    return e


def cap_fasteners(m: dict | None = None):
    """Rear-cap screws (bench: they stay in when the cartridge is swapped): (name, head-top point, axis) in the stage frame."""
    m = m or MOTOR
    L = bolted_levels(m)
    r_s = cap_screw_r(m)
    out = [(f"cap screw {a:+.0f}°", (r_s * math.cos(math.radians(a)), r_s * math.sin(math.radians(a)), L["rear"] + 1.65), (0.0, 0.0, 1.0))
           for a in CAP["ang"]]
    for k in range(0, m["rear_holes"], 2):
        a = 2 * math.pi * k / m["rear_holes"]
        out.append((f"motor screw {math.degrees(a):.0f}°", (m["rear_pcd"] / 2 * math.cos(a), m["rear_pcd"] / 2 * math.sin(a), L["rear"]), (0.0, 0.0, 1.0)))
    return out

def motor_bolted(m: dict | None = None) -> Part:
    m = m or MOTOR
    L = bolted_levels(m)
    return Pos(0, 0, L["motor_face"]) * Cylinder(m["od"] / 2, m["h"], align=MIN)


def oldham_parts(m: dict | None = None):
    """(name, part, colour) of the Oldham drive in the stage frame: clamp hub (on the Ø12 shaft), disc, motor adapter."""
    m = m or MOTOR
    o, L = OLDHAM, bolted_levels(m)
    r = m["oldham"] / 2
    h0, h1 = L["hub"]
    hub = Pos(0, 0, h0) * Cylinder(r, h1 - h0 - o["tongue_h"], align=MIN)
    hub += Pos(0, 0, h1 - o["tongue_h"]) * (Box(2 * r - 2, o["tongue_w"], o["tongue_h"], align=MIN) & Cylinder(r, o["tongue_h"], align=MIN))
    hub -= Pos(0, 0, -200) * Cylinder(6.0, 400, align=MIN)
    d0, d1 = L["disc"]
    disc = Pos(0, 0, d0) * Cylinder(r, d1 - d0, align=MIN)
    disc -= Pos(0, 0, d0) * Box(2 * r + 2, o["tongue_w"] + 0.1, o["slot_d"], align=MIN)
    disc -= Pos(0, 0, d1 - o["slot_d"]) * Rot(0, 0, 90) * Box(2 * r + 2, o["tongue_w"] + 0.1, o["slot_d"], align=MIN)
    a0, a1 = L["adapter"]
    ad = Pos(0, 0, a0) * Cylinder(r, a1 - a0, align=MIN)
    ad += Pos(0, 0, a0 - o["tongue_h"] + o["gap"]) * Rot(0, 0, 90) * (Box(2 * r - 2, o["tongue_w"], o["tongue_h"] - o["gap"], align=MIN)
                                                                      & Cylinder(r, o["tongue_h"], align=MIN))
    for k in range(0, m["out_holes"], 2):                           # 4 × M3 into the motor output, counterbored from the disc side
        a = 2 * math.pi * k / m["out_holes"] + math.pi / 8
        x, y = m["out_pcd"] / 2 * math.cos(a), m["out_pcd"] / 2 * math.sin(a)
        ad -= Pos(x, y, a0) * Cylinder(1.7, 10, align=MIN)
        ad -= Pos(x, y, a0) * Cylinder(3.0, 1.6, align=MIN)
    return [("oldham_hub", hub, "steel"), ("oldham_disc", disc, "dark"), ("drive_adapter", ad, "dark")]


def motor_adapter_bolted(m: dict | None = None) -> Part:
    return dict((n, p) for n, p, _ in oldham_parts(m))["drive_adapter"]


# ---------------------------------------------------------------------------------------------------------------------
# Bought parts per bolted stage (D24 v2, Fable C3/M5/M6, 2026-11-27): for the BOM / M1 kit. Standard designations; sizes
# not on a datasheet are ESTIMATE. Ear-screw count per site: EAR_SITES (elbow 4, pitch 3, yaw 2, lift 2).
def parts_list(m: dict | None = None, site: str = "default"):
    """[(item, spec, qty)] for one bolted cartridge + its small stage, motor m at site."""
    m = m or MOTOR
    d20 = m is MOTOR_8108
    n_ear = len(EAR_SITES[site])
    out = [("actuator", "SteadyWin GIM8108-8 (8:1) with driver; 24 V operation to confirm (D20 gate)" if d20 else "SteadyWin GIM6010-8 with SDC104 driver", 1),
           ("Oldham coupling", f"D{m['oldham']:.0f} clamp-hub Oldham, shaft hub bore 12 H7 with 4 mm keyway, acetal disc "
                               "(the motor-side half is the printed ASA tongue adapter)", 1),
           ("stage shaft", "Ø12 h6 C45 / 1045 ground steel, 4 × 2.5 DIN 6885 keyway from the pulley seat to the end", 1),
           ("keys", f"DIN 6885 A 4 × 4 × {20 if d20 else 16} (pulley) + 4 × 4 × {16 if d20 else 11} (Oldham hub)", 2),
           ("small pulley", f"HTD 5M 20T for 15 mm belt, {'steel C45' if d20 else 'aluminium'}, bore 12 H7 + 4 mm keyway, set screw on the key", 1),
           ("bearings", "6801-2RS (12 × 21 × 5)", 2),
           ("flange ear screws", f"{EAR['screw']}", n_ear),
           ("rear cap", f"{CAP['screw']} × 2; {CAP['motor_screw']} × {m['rear_holes'] // 2} into the motor", 1)]
    if d20:
        out += [("belt", "HTD 5M 15 mm PU steel-cord (Fable M6): 355-5M at the pitch, 380-5M at the elbow", 1),
                ("firmware", "flash torque limit 18 N·m motor-side (sw/bringup.py limit step): D63 disc 1.3×, 6801 s0 1.6× at that limit", 1)]
    return out
