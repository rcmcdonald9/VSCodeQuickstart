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
#     the whole robot. C3 had elbow 4 ears (±40°/±140°), pitch 3 (40°, −30°, −70°), yaw 2 (±145°), lift 2 (±140°); Neon #2
#     finding 2 (2026-11-27): 4 everywhere with gaps ≤ 120° (yaw ±50/±145, pitch4 ±40/160/−120 with the rear pair
#     reversed, lift4 ±40/±140), see EAR_SITES and flange_joint() for the tipping / bolt-load calculation.
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
#     disc → motor-side hub on the motor output (4 × M3, PCD 34). Neon #2 finding 2 (2026-11-27): the motor-side hub was a
#     printed ASA tongue adapter; it is now METAL (MOTOR_HUB: the motor-side hub of the same bought D50 set, faced to a
#     4 mm flange behind its tenon and drilled 4 × Ø3.4 + c'bore on PCD 34 at 45° to the tenon, or the same as a 6061-T6
#     CNC part from drive_adapter). Why not print it (hand calc, ESTIMATE): tenon bearing 3T/(2·h·R²) with h 3.7 = 7.1 MPa
#     at 11 N·m (D50), 9.0 MPa at 22 N·m (D63), root shear 1.3 / 2.6 MPa across the layers, all well under the ASA
#     figures at room temperature, BUT the part sits on the motor's output face (≈ 50–60 °C after a hot soak, ESTIMATE),
#     it slides continuously against the acetal under the ±1 mm eccentric offset (printed-on-POM wear, no data), and its
#     4 mm printed flange is the clamped member of the 4 × M3: a normal 1 kN M3 preload is 68 MPa on the Ø5.5/Ø3.4 head
#     annulus, over ASA's ≈ 50 MPa compressive (ESTIMATE), so it can only take ≈ 0.4 kN, and then friction gives 0.3 × 4 ×
#     0.4 kN × r 17 ≈ 8 N·m < 11 N·m: the peak goes into the screws bearing on printed holes (≈ 22 MPa), in a warm part that
#     creeps. In 6061 the same flange holds 4 × 1 kN (friction ≈ 20 N·m) and only the acetal disc, the part the catalogue
#     rates, wears.
#   - Swap (rnd/assembly_seq.py, R14 (c)/(d)): 4 ear screws (from the cartridge side, or from the pulley side through the
#     opposite wall for reversed ears), the cartridge slides out along its axis off the Oldham disc; belt, pulley, shaft,
#     bearings and the shaft-side Oldham hub stay.
# Stage frame (as small_stage_v3): flange mating plane z = 0 (= the stage wall's cartridge-side face), cartridge +Z.
# All dimensions not on a datasheet are ESTIMATE.
OLDHAM = dict(od=50.0, hub_l=15.0, tongue_h=4.0, tongue_w=10.0, disc_t=10.0, slot_d=4.0, gap=0.3)   # D50 clamp-hub Oldham, 12 mm bore + 4 mm keyway (ESTIMATE; od per motor: MOTOR["oldham"])
ADAPTER_T = 4.0
# Metal motor-side Oldham hub (Neon #2 finding 2): 6061-T6, Ø = the Oldham OD, ADAPTER_T flange + the set's tenon; 4 ×
# DIN 7984 M3 × 6 (low head Ø5.5 × 2) in Ø6.0 × 2.2 counterbores (1.8 mm under the head) into the motor output's threads
# (≥ 4 mm engagement; the output thread depth is not on the drawing, ESTIMATE). Shear per screw at the 11 N·m peak:
# 11 / (4 × 17 mm) ≈ 160 N (22 N·m D20: 320 N) vs ≈ 2 kN for an M3 8.8 in single shear.
MOTOR_HUB = dict(material="6061-T6", clr_d=3.4, cb=(6.0, 2.2), screw="DIN 7984 M3 × 6 (8.8), Loctite 243", n=4)
NOSE_B = dict(r_in=21.5, r_out=24.5, lip=1.5)
FLANGE_B = dict(od=68.0, t=6.0, bolt_r=29.0, insert_d=5.6, insert_l=5.7, clr_d=4.6, csk_d=8.6)   # D24 v1 (legacy: only roll.flange_screw_access still reads it)
FLANGE_ANG = (75.0, -75.0, 115.0, -115.0, 155.0, -155.0)                                         # D24 v1 (legacy, as above)
# D24 v2 ears. Screw ISO 4762 M4 × 12 + ISO 7089 washer (Ø9 × 0.8), nut ISO 4032 M4 (s 7, m 3.2): ISO catalogue sizes.
EAR = dict(t=6.0, boss_r=5.5, clr_d=4.5, head_d=7.0, head_h=4.0, washer=(9.0, 0.8), nut_af=7.0, nut_m=3.2, pocket=3.4,
           driver_r=3.0, driver_clear=1.7, screw="ISO 4762 M4 × 12 + ISO 7089 washer, ISO 4032 M4 nut in a hex pocket",
           rev_boss_r=6.5, rev_hole_d=10.0, rev_screw="ISO 4762 M4 × 16 + ISO 7089 washer from the pulley side, ISO 4032 M4 nut in a side-fed slot in the ear",
           slot_skin=1.3)   # side-fed nut slots: 3.4 tall, 1.3 mm skins (the nut bears on the skin toward the clamped joint: compression)
# Ear angles per site (deg from the stage +X), chosen with rnd/assembly_seq.py's driver probe on the whole robot. A module
# sets EAR_SITE = "<site>" and passes site=EAR_SITE to bolted_stage / bolted_cartridge / wall_a_cuts; the probe reads it.
# Neon review #2 finding 2 (2026-11-27): C3 left yaw and lift with 2 ears over a 70–80° arc and pitch with 3 over 110°, so
# the flange tips about the ear side under the overhung motor and the Oldham side load (flange_joint(), GIM6010-8: yaw /
# lift open at 8.2 N·m with the full 0.75 kN preload but 2.5 N·m at 30 % retained, vs 3.2 N·m applied → 0.8×, and their
# 2-ear friction (6 N·m at 30 %) is under the 11 N·m peak). Rule now: ≥ 3 ears with no gap > 120° between neighbours, or
# ≥ 4 with no gap > 180°. The sites below all have 4 ears with gaps ≤ 120° (≥ 10× tipping at 30 % preload):
#   - REVERSED ears (EAR_REVERSED) where the cartridge side has no driver path (pitch rear: the lift carriage): head on
#     wall A's pulley-side face, nut captive in a side-fed slot in the ear (rev_boss_r lobe), driven from the pulley side
#     through a Ø10 hole in the opposite wall (belt_stage.wall_b_cuts; the screw and washer pass through it);
#   - SIDE-FED wall-A nut slots (EAR_FEED: slot direction from the stage +X) where a pocket on wall A's pulley face cannot
#     be used: yaw (the block posts (40, ±38) and side walls y ±42 sit under the plate there, and the pocket faced down,
#     so the nuts fell into the block on a swap). The slot runs inside the plate to its edge; the nut bears on the 1.3 mm
#     skin between the slot and the ear, in compression.
# "pitch4" / "lift4" are the 4-ear patterns for shoulder.py / upper.py: they need geometry in those files (outer-wall
# holes; wall-A towers + a cover cut, see belt_stage.wall_b_cuts / ear_tower), so the modules switch EAR_SITE when they add
# it; "pitch" / "lift" stay the C3 patterns until then. Elbow (4 at ±40/±140) already met the rule.
EAR_SITES = {
    "elbow": (40.0, -40.0, 140.0, -140.0),   # clear of the cassette posts (stage (10, ±35)), the elbow tube, the −Y seat boss, the housing sides
    "pitch": (40.0, -30.0, -70.0),           # C3: the lift carriage sits 72 mm behind the rear (±140°) pair and the balancer cage over 50–130°; +40° by a ball-end past the AI bay
    "pitch4": (40.0, -40.0, 160.0, -120.0),  # ±40 from the cartridge side; 160 / −120 reversed through the outer wall (the carriage blocks the cartridge side behind −80°/+50°)
    "yaw": (50.0, -50.0, 145.0, -145.0),     # the −X clevis arm's foot covers |y| ≤ 26 for stage x ≥ 34.5 (the ±50 lobes stay at |y| ≥ 31.3); nuts side-fed past the posts
    "lift": (140.0, -140.0),                 # C3: the lift can sits 1.2 mm over the deck and its cover wraps it 3 mm off: only the lower diagonals
    "lift4": (40.0, -40.0, 140.0, -140.0),   # + the upper diagonals: wall A needs r 7.5 towers up to z 244 and the cover 1 mm round them (upper.py, belt_stage.ear_tower)
    "roll_belt": (40.0, -40.0),              # belt-roll fallback: the walls end 37 mm over the stage axis on the −Z side (unchecked)
}
EAR_SITES["default"] = (145.0, -145.0)       # the fallback for a caller that names no site (masses, coupons): the C3 2-ear yaw pattern
EAR_REVERSED = {"pitch4": (160.0, -120.0)}   # driven from the pulley side (head on wall A's pulley face, nut in the ear)
EAR_FEED = {"yaw": {50.0: 90.0, -50.0: -90.0, 145.0: 180.0, -145.0: 180.0}}   # side-fed nut slots inside wall A (slot direction, deg)
# Side exit (R14 (d)): where the cartridge cannot be pulled its own length along its axis (pitch: the lift carriage is
# 12.6 mm behind the rear; lift: the base rim 13 mm behind), the flange, lip and nose are open on one side (a U-slot as
# wide as the Oldham disc, up to just under the motor-side adapter). The cartridge then lifts 4.5 mm (adapter tongue out
# of the disc, lip out of the counterbore) and slides off sideways AWAY from the opening, so the shaft-side hub and the
# disc (which stays on it) leave through the slot. Opening angle from the stage +X (pitch: +Y = up, the cartridge drops
# out downward; lift: −X = down, it comes out upward); the ears keep ≥ 40° clear of it.
EXIT_SITES = {"pitch": 90.0, "lift": 180.0, "lift4": 180.0}
# D26 (owner, 2026-11-27): the arm sites drive their pulley straight from the motor output (belt_stage.small_stage_v4: a
# turned stub on the output flange, a 6001 in wall B, a slotted idler); the lift sites keep the Oldham (v3) until
# upper.py moves (the edit is in the D26 report). With the pulley on the motor nothing static passes through the
# flange, so the flange is whole again; where the cartridge cannot be pulled its stub's length (pitch: the lift carriage
# 12.6 mm behind), wall A gets a U-slot instead (WALL_EXIT: the direction the pulley and stub leave, from the stage +X,
# after 8.5 mm along the axis frees the 6001 from wall B).
DIRECT_SITES = {"pitch4", "elbow", "yaw", "roll_belt"}   # + "lift" / "lift4" once upper.py moves the lift to the direct stage
WALL_EXIT = {"pitch4": 270.0, "lift": 0.0, "lift4": 0.0}   # lift: up (stage +X), once its open belt is unclamped at the carriage and lifted off the 20T
DIRECT_LIFT = 8.5                            # axial travel that frees the 6001 (8 wide, flush with wall B's pulley face) + 0.5
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
            dict(od=m["flange"][0], t=EAR["t"], bolt_r=ear_r(m), ang=EAR_SITES[site], clr_d=EAR["clr_d"],
                 rev=EAR_REVERSED.get(site, ()), feed=EAR_FEED.get(site, {})))


def ear_boss_r(site: str, a: float) -> float:
    """Ear lobe radius: rev_boss_r round a reversed ear's side-fed nut slot, boss_r elsewhere."""
    return EAR["rev_boss_r"] if a in EAR_REVERSED.get(site, ()) else EAR["boss_r"]


def ear_fasteners(m: dict | None = None, site: str = "default"):
    """[(name, head-top point, driver axis)] of the flange ear screws in the stage frame (axis +Z: out on the cartridge side;
    a reversed ear's head sits on wall A's pulley face, z −6, and is driven along −Z through the opposite wall)."""
    _, f = nose_flange(m, site)
    z = f["t"] + EAR["washer"][1] + EAR["head_h"]
    zr = -6.0 - EAR["washer"][1] - EAR["head_h"]
    out = []
    for a in f["ang"]:
        x, y = f["bolt_r"] * math.cos(math.radians(a)), f["bolt_r"] * math.sin(math.radians(a))
        if a in f["rev"]:
            out.append((f"ear screw {a:+.0f}° (from the pulley side)", (x, y, zr), (0.0, 0.0, -1.0)))
        else:
            out.append((f"ear screw {a:+.0f}°", (x, y, z), (0.0, 0.0, 1.0)))
    return out


def ear_gaps(site: str):
    """Angular gaps (deg) between neighbouring ears, largest first."""
    a = sorted(x % 360.0 for x in EAR_SITES[site])
    return sorted(((a[(k + 1) % len(a)] - a[k]) % 360.0 or 360.0 for k in range(len(a))), reverse=True)


def ear_rule_ok(site: str) -> bool:
    """Neon #2 finding 2: ≥ 3 ears with every gap ≤ 120° (spread ≥ 240°), or ≥ 4 with every gap ≤ 180°."""
    n, g = len(EAR_SITES[site]), ear_gaps(site)[0]
    return (n >= 3 and g <= 120.0 + 1e-6) or (n >= 4 and g <= 180.0 + 1e-6)


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
        fl += _lobe(f["bolt_r"], a, ear_boss_r(site, a), f["t"])
    body += fl
    body += Pos(0, 0, -n["lip"]) * Cylinder(n["r_out"], n["lip"], align=MIN)             # centring lip (spigot)
    body -= Pos(0, 0, -5) * Cylinder(n["r_in"], L["motor_face"] + 5, align=MIN)          # coupling bore through flange, nose, step
    for a in f["ang"]:
        t = math.radians(a)
        body -= Pos(f["bolt_r"] * math.cos(t), f["bolt_r"] * math.sin(t), -1) * Cylinder(EAR["clr_d"] / 2, f["t"] + 2, align=MIN)
        if a in f["rev"]:                                 # reversed ear: side-fed slot for the captive M4 nut, open at the lobe tip
            sk, h = EAR["slot_skin"], f["t"] - 2 * EAR["slot_skin"]
            r0, r1 = f["bolt_r"] - EAR["nut_af"] / 2 - 0.05, f["bolt_r"] + ear_boss_r(site, a) + 1.0
            body -= Rot(0, 0, a) * Pos((r0 + r1) / 2, 0, sk) * Box(r1 - r0, EAR["nut_af"] + 0.1, h, align=(Align.CENTER, Align.CENTER, Align.MIN))
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
        e += _lobe(f["bolt_r"], a, ear_boss_r(site, a) + clr, h, z0)
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
    h = MOTOR_HUB
    for k in range(0, m["out_holes"], 2):                           # 4 × M3 into the motor output, counterbored from the disc side
        a = 2 * math.pi * k / m["out_holes"] + math.pi / 4          # 45° off the tenon (along 90°): the Ø6 counterbores stay 4 mm clear of it
        x, y = m["out_pcd"] / 2 * math.cos(a), m["out_pcd"] / 2 * math.sin(a)
        ad -= Pos(x, y, a0) * Cylinder(h["clr_d"] / 2, 10, align=MIN)
        ad -= Pos(x, y, a0) * Cylinder(h["cb"][0] / 2, h["cb"][1], align=MIN)
    # Neon #2 finding 2: the motor-side half is now a metal part (6061-T6, MOTOR_HUB), no longer printed ASA
    return [("oldham_hub", hub, "steel"), ("oldham_disc", disc, "dark"), ("drive_adapter", ad, "steel")]


def motor_adapter_bolted(m: dict | None = None) -> Part:
    return dict((n, p) for n, p, _ in oldham_parts(m))["drive_adapter"]


# ---------------------------------------------------------------------------------------------------------------------
# Bought parts per bolted stage (D24 v2, Fable C3/M5/M6, 2026-11-27): for the BOM / M1 kit. Standard designations; sizes
# not on a datasheet are ESTIMATE. Ear-screw count per site: EAR_SITES (elbow 4, pitch 3, yaw 2, lift 2).
def parts_list_direct(m: dict | None = None, site: str = "elbow", idler: bool = False):
    """D26: [(item, spec, qty)] for one bolted cartridge + its DIRECT stage (belt_stage.small_stage_v4), motor m at site;
    idler=True adds the pitch's back-side slotted idler (rnd/belts.py IDLERS)."""
    from . import belt_stage as bs
    m = m or MOTOR
    d20 = m is MOTOR_8108
    n_rev = len(EAR_REVERSED.get(site, ()))
    n_ear = len(EAR_SITES[site]) - n_rev
    out = [("actuator", "SteadyWin GIM8108-8 (8:1) with driver; 24 V operation to confirm (D20 gate)" if d20 else "SteadyWin GIM6010-8 with SDC104 driver", 1),
           ("drive stub", f"turned {bs.STUB['material']}: Ø{bs.STUB['flange_od']:.0f} × {bs.STUB['t']:.0f} flange, 8 × Ø3.4 + Ø6.0 × 2.2 c'bore on PCD "
                          f"{m['out_pcd']:.0f} (ESTIMATE until coupon_motor), Ø{bs.STUB['pilot_d']:.0f} pilot (ESTIMATE), Ø12 h6 × 63 shaft, k6 seat for the 6001 at its end", 1),
           ("stub screws", f"DIN 7984 M3 × 6 {'10.9' if d20 else '8.8'}, Loctite 243", 8),
           ("key", f"DIN 6885 A 4 × 4 × {20 if d20 else 16} (pulley)", 1),
           ("small pulley", f"HTD 5M 20T for 15 mm belt, {'steel C45' if d20 else 'aluminium'}, bore 12 H7 + 4 mm keyway, set screw on the key; "
                            "hub-side flange turned off (motor-side flange only: the belt slides off in mesh, belts.INSTALL_SLACK)", 1),
           ("bearing", f"{bs.BRG_B['name']} (12 × 28 × 8) pressed on the stub end; outer ring slip fit in the printed bushing", 1),
           ("bushing adhesive", f"wall-B bushing bonded after the cartridge is bolted: {bs.BUSH_B['adhesive']} (from the epoxy line)", 1),
           ("flange ear screws", f"{EAR['screw']}", n_ear),
           ("rear cap", f"{CAP['screw']} × 2; {CAP['motor_screw']} × {m['rear_holes'] // 2} into the motor", 1)]
    if n_rev:
        out.insert(-1, ("flange ear screws (reversed)", EAR["rev_screw"], n_rev))
    if idler:
        out += [("idler bearings", "625-2RS (5 × 16 × 5)", 2),
                ("idler axle", "ISO 4762 M5 × 40 8.8 (plain shank under the bearings) + ISO 4032 M5 nut (captive in wall A) + 2 aluminium Ø8 × 5.3 round spacers", 1),
                ("idler jack screw", "ISO 4762 M4 × 20 + M4 nut in the wall-B lug (pushes the axle along its slot)", 1),
                ("idler roller", "printed PETG-CF flat roller Ø25 × 16 with 1.5 mm flanges (print plan)", 1)]
    if d20:
        out += [("belt", "HTD 5M 15 mm PU steel-cord (Fable M6): 355-5M at the pitch, 380-5M at the elbow", 1),
                ("firmware", "flash torque limit 18 N·m motor-side (sw/bringup.py limit step): 6001 2.0×, stub 8 × M3 1.7× at that limit", 1)]
    return out


def parts_list(m: dict | None = None, site: str = "default"):
    """[(item, spec, qty)] for one bolted cartridge + its small stage, motor m at site (D24 v3 Oldham stage; D26 arm sites:
    parts_list_direct)."""
    if site in DIRECT_SITES:
        return parts_list_direct(m, site, idler=site.startswith("pitch"))
    m = m or MOTOR
    d20 = m is MOTOR_8108
    n_rev = len(EAR_REVERSED.get(site, ()))
    n_ear = len(EAR_SITES[site]) - n_rev
    D = f"D{m['oldham']:.0f}"
    out = [("actuator", "SteadyWin GIM8108-8 (8:1) with driver; 24 V operation to confirm (D20 gate)" if d20 else "SteadyWin GIM6010-8 with SDC104 driver", 1),
           ("Oldham coupling", f"{D} 3-piece Oldham set (2 aluminium hubs + acetal disc), rated ≥ {'25' if d20 else '14'} N·m; "
                               "shaft-side hub: clamp type, bore 12 H7 with a 4 mm DIN 6885 keyway", 1),
           ("Oldham motor-side hub (metal)", f"the set's 2nd hub reworked, or a CNC part from the drive_adapter STEP: {MOTOR_HUB['material']}, "
                                             f"{D} × {ADAPTER_T:.0f} mm flange behind the tenon, 4 × Ø3.4 + Ø6.0 × 2.2 c'bore on PCD {m['out_pcd']:.0f} "
                                             "(ESTIMATE until coupon_motor) at 45° to the tenon", 1),
           ("Oldham hub screws", f"{MOTOR_HUB['screw']}", MOTOR_HUB["n"]),
           ("stage shaft", "Ø12 h6 C45 / 1045 ground steel, 4 × 2.5 DIN 6885 keyway from the pulley seat to the end", 1),
           ("keys", f"DIN 6885 A 4 × 4 × {20 if d20 else 16} (pulley) + 4 × 4 × {16 if d20 else 11} (Oldham hub)", 2),
           ("small pulley", f"HTD 5M 20T for 15 mm belt, {'steel C45' if d20 else 'aluminium'}, bore 12 H7 + 4 mm keyway, set screw on the key", 1),
           ("bearings", "6801-2RS (12 × 21 × 5)", 2),
           ("flange ear screws", f"{EAR['screw']}", n_ear),
           ("rear cap", f"{CAP['screw']} × 2; {CAP['motor_screw']} × {m['rear_holes'] // 2} into the motor", 1)]
    if n_rev:
        out.insert(-1, ("flange ear screws (reversed)", EAR["rev_screw"], n_rev))
    if d20:
        out += [("belt", "HTD 5M 15 mm PU steel-cord (Fable M6): 355-5M at the pitch, 380-5M at the elbow", 1),
                ("firmware", "flash torque limit 18 N·m motor-side (sw/bringup.py limit step): D63 disc 1.3×, 6801 s0 1.6× at that limit", 1)]
    return out


# ---------------------------------------------------------------------------------------------------------------------
# Flange-joint tipping and bolt loads (Neon review #2 finding 2, 2026-11-27). Hand calc, all inputs ESTIMATE unless noted.
# Loads on the ear joint (the belt pull never reaches it: it goes from the pulley into the 6801s in the walls):
#   - torque T = actuator peak (GIM6010-8 11 N·m, GIM8108-8 22 N·m stall; rnd/specs/actuators.yaml), by friction;
#   - overhung cartridge: motor (388 / 396 g, actuators.yaml) + printed sleeve and cap (≈ 70 / 90 g from the CAD volume,
#     ASA at 80 %) + metal motor-side hub (≈ 26 / 40 g) at the motor's mid-length (z 45 / 50 mm from the flange plane),
#     × FLANGE_LOAD["g"] (3 g: carry bumps and set-downs, ESTIMATE), in any direction (the arm turns every cartridge);
#   - Oldham side load: with the ±1 mm eccentric offset the disc slides in both slots; the friction on the motor-side
#     tenon is ≈ μ·T / r_c (μ 0.2 acetal on aluminium, r_c = D/3 the contact-force radius; ESTIMATE) at the tenon's
#     mid-height (z 19.25 mm): 132 N × 19.25 mm = 2.5 N·m on the GIM6010 at its peak.
# Tipping: the flange (disc + ear lobes) pivots about its edge line in the worst direction θ; the ears resist with their
# preload × distance from that line, M_open(θ) = F_p · Σ d_i(θ) (the spigot lip is ignored). Bolt load once tipped
# (rigid flange, all the moment on the ears): F_max = F_p + M · d_max / Σ d². Friction torque μ 0.3 · n · F_p · r_ear.
# "30 %" = the preload left after printed-joint creep (Neon #2 assumption 1 argues it can go lower; ESTIMATE).
FLANGE_LOAD = dict(preload=750.0, retained=0.3, g=3.0, mu_face=0.3, mu_oldham=0.2, m_print=(0.070, 0.090), m_hub=(0.026, 0.040),
                   screw_proof=580.0 * 8.78, nut_area=26.5)   # M4 8.8 proof 580 MPa × As 8.78 mm² (ISO 898-1) = 5.1 kN; ISO 4032 M4 bearing ring mm²


def direct_brg_load(m: dict | None = None, T: float | None = None):
    """D26 direct stage: radial load on the motor's output bearing at torque T (default the actuator peak): the small
    pulley's shaft load F + 2·T0 (rnd/lift.py model: tight side F + T0, slack side T0, T0 = 0.25 F) times the motor
    bearing's share of the stub span (belt_stage.support_split). Returns (motor bearing N, 6001 N, shaft load N, z of the
    motor bearing in the stage frame)."""
    from . import belt_stage as bs
    m = m or MOTOR
    T = (22.0 if m is MOTOR_8108 else 11.0) if T is None else T
    F = T * 1000 / (20 * bs.PITCH / math.pi / 2)
    shaft = F * (1 + 2 * 0.25)
    sm, sb, _, _ = bs.support_split(m)
    return sm * shaft, sb * shaft, shaft, bs._v4_levels(m)["motor_brg"]


def flange_joint(m: dict | None = None, angles=None, site: str = "default", retained: float = 1.0, direct: bool | None = None):
    """Tipping / bolt-load check of one ear pattern for motor m: angles (deg) given, or the site's (with its lobe sizes).
    direct (default: the site is in DIRECT_SITES): D26, the motor's output bearing carries its share of the belt pull, which
    the cartridge passes to wall A (shear through the centring lip, a moment on the ears); no Oldham side load."""
    m = m or MOTOR
    d20 = m is MOTOR_8108
    L = FLANGE_LOAD
    direct = (angles is None and site in DIRECT_SITES) if direct is None else direct
    ang = tuple(EAR_SITES[site] if angles is None else angles)
    r, od = ear_r(m), m["flange"][0]
    T = 22.0 if d20 else 11.0
    mass = (0.396 if d20 else 0.388) + L["m_print"][d20] + L["m_hub"][d20]
    lv = bolted_levels(m)
    z_cg = lv["motor_face"] + m["h"] / 2
    if direct:                                                                       # stub (≈ 0.1 kg C45) half on the motor
        F_r, z_old = direct_brg_load(m)[0], direct_brg_load(m)[3]
        mass += 0.05
    else:
        z_old = lv["adapter"][0] - (OLDHAM["tongue_h"] - OLDHAM["gap"]) / 2
        F_r = L["mu_oldham"] * T * 1000 / (m["oldham"] / 3)
    M_app = (mass * 9.81 * L["g"] * z_cg + F_r * z_old) / 1000                       # N·m
    Fp = L["preload"] * retained
    pts = [(r * math.cos(math.radians(a)), r * math.sin(math.radians(a)), ear_boss_r(site, a) if angles is None else EAR["boss_r"]) for a in ang]
    worst = None
    for k in range(360):
        c, s = math.cos(math.radians(k)), math.sin(math.radians(k))
        h = max([od / 2] + [x * c + y * s + rb for x, y, rb in pts])
        d = [h - (x * c + y * s) for x, y, _ in pts]
        if worst is None or sum(d) < sum(worst[1]):
            worst = (k, d)
    k, d = worst
    Mo = Fp * sum(d) / 1000
    F_max = Fp + M_app * 1000 * max(d) / sum(x * x for x in d)
    a = sorted(x % 360 for x in ang)
    gap = max(((a[(i + 1) % len(a)] - a[i]) % 360 or 360) for i in range(len(a)))
    T_fric = L["mu_face"] * len(ang) * Fp * r / 1000
    return dict(n=len(ang), angles=ang, gap=gap, rule=(len(ang) >= 3 and gap <= 120.001) or (len(ang) >= 4 and gap <= 180.001),
                T=T, mass=mass, direct=direct, z_cg=z_cg, F_r=F_r, M_app=M_app, preload=Fp, M_open=Mo, theta=k, sf_tip=Mo / M_app,
                F_max=F_max, sf_bolt=L["screw_proof"] / F_max, p_nut=F_max / L["nut_area"], T_fric=T_fric, sf_T=T_fric / T)


C3_PATTERNS = {"pitch": (40.0, -30.0, -70.0), "yaw": (145.0, -145.0), "lift": (140.0, -140.0), "elbow": (40.0, -40.0, 140.0, -140.0)}
NEW_SITES = {"pitch": "pitch4", "yaw": "yaw", "lift": "lift4", "elbow": "elbow"}   # "now": the arm sites are D26 direct stages


def flange_rows():
    """(joint, motor, [before full, before 30 %], [after full, after 30 %]) for the report (rnd/belts.py)."""
    rows = []
    for j, mot in (("pitch", MOTOR), ("elbow", MOTOR), ("yaw", MOTOR), ("lift", MOTOR), ("pitch", MOTOR_8108), ("elbow", MOTOR_8108)):
        b = [flange_joint(mot, C3_PATTERNS[j], retained=x) for x in (1.0, FLANGE_LOAD["retained"])]
        a = [flange_joint(mot, None, NEW_SITES[j], retained=x) for x in (1.0, FLANGE_LOAD["retained"])]
        rows.append((j, "GIM8108-8 (D20)" if mot is MOTOR_8108 else "GIM6010-8", b, a))
    return rows


if __name__ == "__main__":
    def _f(r):
        return (f"{r[0]['n']} ears, gap {r[0]['gap']:.0f}° ({'ok' if r[0]['rule'] else 'FAILS'}): M_app {r[0]['M_app']:.2f} N·m, "
                f"M_open {r[0]['M_open']:.1f} / {r[1]['M_open']:.1f} (30 %) → {r[0]['sf_tip']:.1f}× / {r[1]['sf_tip']:.1f}×, "
                f"F_max {r[0]['F_max']:.0f} N, T_fric {r[0]['T_fric']:.0f} / {r[1]['T_fric']:.0f} N·m vs {r[0]['T']:.0f}")
    for j, mot, b, a in flange_rows():
        print(f"{j:6s} {mot:16s} before: {_f(b)}\n{'':23s} after:  {_f(a)}")
