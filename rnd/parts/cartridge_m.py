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
             rear_holes=6, rear_pcd=57.0)         # 6×M3 on the housing (count from the drawing; PCD ESTIMATE)
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
# D24 (owner, 2026-11-27): bolted cartridge M, no KX socket. Used by the arm belt stages (shoulder pitch, elbow) through
# belt_stage.small_stage_v3; the lift (upper.py), wheels (base.py) and roll/yaw (roll.py) still use can() + KX74 until
# their owners switch.
#   - Sleeve: Ø86.6 around the motor (as before), a front step, a Ø49 nose around the Oldham coupling and a Ø68 flange
#     with 6 × M4 heat-set inserts (r 29). The nose lip (Ø49 × 1.5) is the centring spigot in the stage wall's counterbore.
#     The motor goes in from the rear (the step holds its front face); a separate rear cap bolts to the motor's housing
#     holes (3 × M3, PCD 57) and is held in the sleeve by 3 radial M3 into inserts in its rim.
#   - 6 × M4 countersunk screws come from the pulley side of the stage wall (flush heads, no belt or pulley clearance
#     needed) into the flange inserts. Torque only: GIM peak 11 N·m / (6 × 29 mm) ≈ 63 N per screw; the belt pull stays
#     in the stage bearings. The bolt angles (FLANGE_ANG, from the belt direction) keep the key path clear of the belt
#     strands and the big pulley, so a swap is 6 screws with the belt on.
#   - Drive: an Oldham coupling (blind axial mate, absorbs the ±1 mm the eccentric bushings move the stage shaft):
#     aluminium clamp hub on the Ø12 stage shaft (off the shelf, Ø40) → acetal disc → printed ASA tongue adapter on the
#     motor output (4 × M3, PCD 34). Tongue bearing ≈ 3T/(2·h·R²) ≈ 13 MPa at the 11 N·m peak (hand calc; coupon).
# Stage frame (as small_stage_v3): flange mating plane z = 0 (= the stage wall's cartridge-side face), cartridge +Z.
# All dimensions not on a datasheet are ESTIMATE.
OLDHAM = dict(od=40.0, hub_l=15.0, tongue_h=4.0, tongue_w=10.0, disc_t=10.0, slot_d=4.0, gap=0.3)   # Ø40 clamp-hub Oldham, 12 mm bore (ESTIMATE)
ADAPTER_T = 4.0
NOSE_B = dict(r_in=21.5, r_out=24.5, lip=1.5)
FLANGE_B = dict(od=68.0, t=6.0, bolt_r=29.0, insert_d=5.6, insert_l=5.7, clr_d=4.6, csk_d=8.6)
FLANGE_ANG = (75.0, -75.0, 115.0, -115.0, 155.0, -155.0)   # deg from the stage +X (toward the big pulley)
STEP_T = 3.0
CAP_T = FLOOR


def bolted_levels() -> dict:
    """Stage-frame z levels of the Oldham drive and the sleeve (flange plane z = 0)."""
    o = OLDHAM
    hub = (-0.5, -0.5 + o["hub_l"])                         # clamp hub, tongue in its top tongue_h
    disc = (hub[1] + o["gap"] - o["tongue_h"], hub[1] + o["gap"] - o["tongue_h"] + o["disc_t"])
    ad = (disc[1] + o["gap"], disc[1] + o["gap"] + ADAPTER_T)  # adapter base; its tongue hangs into the disc's upper slot
    face = ad[1]                                            # motor output face
    return dict(hub=hub, disc=disc, adapter=ad, motor_face=face, step=(face - STEP_T, face), flange=(0.0, FLANGE_B["t"]),
                rear=face + MOTOR["h"] + CAP_T)


def stack_height_bolted() -> float:
    """Can rear face → flange mating plane (the bolted analogue of stack_height())."""
    return bolted_levels()["rear"]


def can_bolted() -> Part:
    """D24 sleeve in the STAGE frame (flange plane z = 0, motor toward +Z): nose lip, flange, nose, step, motor sleeve."""
    L = bolted_levels()
    n, f = NOSE_B, FLANGE_B
    r_in, r_out = MOTOR["od"] / 2 + 0.3, MOTOR["od"] / 2 + WALL + 0.3
    z_rear = L["rear"]
    body = Pos(0, 0, L["step"][0]) * Cylinder(r_out, z_rear - L["step"][0], align=MIN)
    body -= Pos(0, 0, L["motor_face"]) * Cylinder(r_in, z_rear, align=MIN)              # motor bore, open at the rear
    body = fillet(body.edges().group_by(Axis.Z)[0].sort_by(SortBy.RADIUS)[-1], 2.0)
    body += Pos(0, 0, f["t"]) * Cylinder(n["r_out"], L["step"][0] - f["t"] + 0.5, align=MIN)   # nose
    body += Cylinder(f["od"] / 2, f["t"], align=MIN)                                     # flange
    body += Pos(0, 0, -n["lip"]) * Cylinder(n["r_out"], n["lip"], align=MIN)             # centring lip (spigot)
    body -= Pos(0, 0, -5) * Cylinder(n["r_in"], L["motor_face"] + 5, align=MIN)          # coupling bore through flange, nose, step
    for a in FLANGE_ANG:
        t = math.radians(a)
        body -= Pos(f["bolt_r"] * math.cos(t), f["bolt_r"] * math.sin(t), 0) * Cylinder(f["insert_d"] / 2 - 0.1, f["insert_l"] + 0.3, align=MIN)
    body -= Pos(r_out, 0, (L["motor_face"] + z_rear) / 2) * Box(12, 16, MOTOR["h"] - 8)   # side window: CAN + power pigtail
    body -= Pos(0, 0, z_rear - 8) * (Cylinder(r_out + 1, 2) - Cylinder(r_out - 1.2, 2))     # light-ring groove
    for k in range(3):                                                                    # radial M3 for the rear cap
        a = 2 * math.pi * k / 3 + math.pi / 6
        body -= Pos(r_out * math.cos(a), r_out * math.sin(a), z_rear - CAP_T / 2) * Rot(0, 90, math.degrees(a)) * Cylinder(1.7, 8)
    return body


def can_cap_bolted() -> Part:
    """Rear cap of the bolted can (stage frame): carries the motor (3 × M3 at PCD 57), held by 3 radial M3."""
    L = bolted_levels()
    z0 = L["rear"] - CAP_T
    c = Pos(0, 0, z0) * Cylinder(MOTOR["od"] / 2 + 0.2, CAP_T, align=MIN)
    for k in range(0, MOTOR["rear_holes"], 2):
        a = 2 * math.pi * k / MOTOR["rear_holes"]
        c -= Pos(MOTOR["rear_pcd"] / 2 * math.cos(a), MOTOR["rear_pcd"] / 2 * math.sin(a), z0) * Cylinder(1.7, 20)
    c -= Pos(0, 0, z0) * Cylinder(13, 20)                                                 # rear vent / cable pass-through
    return c


def motor_bolted() -> Part:
    L = bolted_levels()
    return Pos(0, 0, L["motor_face"]) * Cylinder(MOTOR["od"] / 2, MOTOR["h"], align=MIN)


def oldham_parts():
    """(name, part, colour) of the Oldham drive in the stage frame: clamp hub (on the Ø12 shaft), disc, motor adapter."""
    o, L = OLDHAM, bolted_levels()
    r = o["od"] / 2
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
    for k in range(0, MOTOR["out_holes"], 2):                       # 4 × M3 into the motor output, counterbored from the disc side
        a = 2 * math.pi * k / MOTOR["out_holes"] + math.pi / 8
        x, y = MOTOR["out_pcd"] / 2 * math.cos(a), MOTOR["out_pcd"] / 2 * math.sin(a)
        ad -= Pos(x, y, a0) * Cylinder(1.7, 10, align=MIN)
        ad -= Pos(x, y, a0) * Cylinder(3.0, 1.6, align=MIN)
    return [("oldham_hub", hub, "steel"), ("oldham_disc", disc, "dark"), ("drive_adapter", ad, "dark")]


def motor_adapter_bolted() -> Part:
    return dict((n, p) for n, p, _ in oldham_parts())["drive_adapter"]
