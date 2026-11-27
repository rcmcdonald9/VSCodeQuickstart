"""Printed HTD 5M pulleys and the small-pulley hub (KX74 socket + straddle-mounted pulley on 2 bearings).

Load path: the cartridge (torque only) locks into the hub's KX74 face; belt pull goes into two 61804 bearings
(20×32×7) that sit in the bone walls on both sides of the pulley. Tooth form is an approximation of HTD 5M
(round groove, depth 2.06 mm, pitch-line offset 0.57 mm): print a test pulley and check it against a real belt.
"""
from __future__ import annotations

import math

from build123d import Align, Box, Cylinder, Part, Pos, Rot

from .kx import KX74, hub_pip as hub

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
PITCH, PLD, DEPTH, GROOVE_R = 5.0, 0.5715, 2.06, 1.60
BRG_6004 = dict(d=20.0, D=32.0, B=7.0)   # 61804 thin-section (C0 ≈ 2.3 kN, ESTIMATE); name kept for callers


def pulley(teeth: int, width: float = 16.0, flange: bool = True, bore: float = 0.0) -> Part:
    pd = teeth * PITCH / math.pi
    od = pd - 2 * PLD
    p = Cylinder(od / 2, width, align=MIN)
    rc = od / 2 - DEPTH + GROOVE_R
    for k in range(teeth):
        a = 2 * math.pi * k / teeth
        p -= Pos(rc * math.cos(a), rc * math.sin(a), 0) * Cylinder(GROOVE_R, width, align=MIN)
        p -= Pos((rc + GROOVE_R) * math.cos(a), (rc + GROOVE_R) * math.sin(a), 0) * Rot(0, 0, math.degrees(a)) * Box(1.6, 2 * GROOVE_R, width, align=MIN)
    if flange:
        for z in (-1.2, width):
            p += Pos(0, 0, z) * Cylinder(od / 2 + 2.0, 1.2, align=MIN)
    if bore:
        p -= Cylinder(bore / 2, 200)
    return p


def small_hub(teeth: int = 20, belt_w: float = 15.0) -> Part:
    """KX74 hub face on top (z ≥ 0 side is the cartridge), then a Ø20 journal, pulley, Ø20 journal (straddle)."""
    w = belt_w + 1.0
    b = BRG_6004
    top_j = Pos(0, 0, -b["B"] - 1) * Cylinder(b["d"] / 2, b["B"] + 1, align=MIN)          # journal under the hub
    pul = Pos(0, 0, -b["B"] - 1 - 1.2 - w) * pulley(teeth, w)
    bot_j = Pos(0, 0, -b["B"] - 1 - 1.2 - w - 1.2 - b["B"]) * Cylinder(b["d"] / 2, b["B"], align=MIN)
    h = Pos(0, 0, -0.0) * hub(KX74)
    hz = h.bounding_box().min.Z
    return Pos(0, 0, 0) * h + Pos(0, 0, hz) * (top_j + pul + bot_j) - Pos(0, 0, hz - 200) * Cylinder(4.2, 400, align=MIN)


def bearings_dummy(z_top: float, belt_w: float = 15.0) -> Part:
    b = BRG_6004
    w = belt_w + 1.0
    ring = Cylinder(b["D"] / 2, b["B"], align=MIN) - Cylinder(b["d"] / 2, b["B"], align=MIN)
    z1 = z_top - b["B"] - 1
    z2 = z1 - 1.2 - w - 1.2 - b["B"]
    return Pos(0, 0, z1) * ring + Pos(0, 0, z2) * ring


def belt_loop(pd_small: float, pd_big: float, centre: float, width: float = 15.0, t: float = 3.6, exact: bool = False) -> Part:
    """Belt as a solid band around two pitch circles (for renders): hull of two discs minus the inner hull.
    exact=True (2026-11-27): the hull between the discs is the real tangent quadrilateral. The default box (half-width =
    the mean radius) bulges past the strands near the small pulley: with a 20T:60T it stood 9 mm proud of the real strand
    at the elbow cassette posts."""
    def hull_exact(r1, r2):
        from build123d import Polygon, extrude
        s = (r2 - r1) / centre
        c = math.sqrt(1 - s * s)
        pts = [(-r1 * s, -r1 * c), (centre - r2 * s, -r2 * c), (centre - r2 * s, r2 * c), (-r1 * s, r1 * c)]   # CCW: extrudes +Z
        body = Cylinder(r1, width, align=MIN) + Pos(centre, 0, 0) * Cylinder(r2, width, align=MIN)
        return body + extrude(Polygon(*pts, align=None), width)

    def hull(r1, r2):
        if exact:
            return hull_exact(r1, r2)
        a = math.asin((r2 - r1) / centre)
        L = math.hypot(centre, 0)
        body = Cylinder(r1, width, align=MIN) + Pos(centre, 0, 0) * Cylinder(r2, width, align=MIN)
        # quadrilateral between tangent points
        mid = (r1 + r2) / 2
        body += Pos(centre / 2, 0, 0) * Box(centre, 2 * mid * math.cos(a) * 0.999, width, align=MIN)
        return body
    return hull(pd_small / 2 + t / 2, pd_big / 2 + t / 2) - hull(pd_small / 2 - t / 2, pd_big / 2 - t / 2)


ECC = dict(od=38.0, e=1.0, t=7.0)   # printed eccentric bushing around each 61804: rotate to tension the belt, 1 lock screw


def eccentric_bushing() -> Part:
    """Bearing seat Ø32 offset by e inside an Ø38 ring with a lock tab: ±1 mm centre-distance adjustment (≈ ±2 mm belt
    length) without slots or a tensioning screw. ~5 min per stage instead of ~15 (ESTIMATE)."""
    e = ECC
    r = Cylinder(e["od"] / 2, e["t"], align=MIN) - Pos(e["e"], 0, 0) * Cylinder(BRG_6004["D"] / 2, e["t"], align=MIN)
    r += Pos(e["od"] / 2 + 3, 0, 0) * (Box(10, 8, 2.5, align=MIN) - Pos(2, 0, 0) * Cylinder(1.7, 3, align=MIN))
    return r


# ---------------------------------------------------------------------------------------------------------------------
# D13 small-pulley stage v2 (loop 37; external review #1 finding 1): assemblable from standard parts.
#   steel Ø12 shaft · 2 × 6801 (12×21×5) in eccentric bushings (wall ≥ 7 mm, was 2 mm) · off-the-shelf aluminium HTD 5M
#   pulley (set screws on a flat) · rigid flange coupling (clamp on the shaft, 4 × M4 into heat-set inserts in a pocket
#   under the printed KX74 hub: ≈ 5 MPa bearing at 12.7 N·m).
# Assembly: flange → hub (4 × M4); shaft into wall A's bearing, pulley slid on between the walls, shaft on into wall B's
# bearing; coupling clamped on the shaft end; belt on; tension with the bushings.
# Stage frame: KX face at z = 0, cartridge +Z, stack toward −Z. Part dimensions marked ESTIMATE until datasheets are in.
SHAFT_D = 12.0
BRG_6801 = dict(d=12.0, D=21.0, B=5.0)            # C0 ≈ 1.0 kN (ESTIMATE); 2 per stage straddle the pulley: ≈ 0.35 kN each at the 691 N peak
FLANGE = dict(od=32.0, t=6.0, bolt_r=12.0, clamp_od=22.0, clamp_l=12.0)   # rigid flange coupling, 12 mm bore (ESTIMATE)
AL_PULLEY = dict(flange_od=38.0, body_l=20.0, hub_d=24.0, hub_l=7.0)      # HTD 5M, 15 mm belt, 20T/18T (ESTIMATE)
# Fable M5 (2026-11-27): set screws on a flat carry ≈ 12 N·m vs the 11 N·m GIM6010 peak (rnd/belts.py shaft_joints), so
# the pulley and the shaft-side Oldham hub are keyed: 12 H7 bore + DIN 6885 A 4 × 4 keyway, one milled 4 × 2.5 keyway in
# the Ø12 shaft from the pulley seat to the end, set screw over the key for axial location (not modelled: no clearance
# changes). D20 (GIM8108-8, 22 N·m): steel 20T (C45) instead of aluminium, key L 20.
ECC_V2 = dict(od=38.0, e=1.0, t=5.0)


def _v2_stack():
    from .kx import KX74, hub_pip
    hub_h = -hub_pip(KX74).bounding_box().min.Z                 # 12.2
    z = -hub_h - FLANGE["clamp_l"]                              # bottom of the coupling's clamp hub
    jA = (z - 0.5 - BRG_6801["B"], z - 0.5)
    p0 = jA[0] - 0.5                                            # top of the pulley
    pul = (p0 - AL_PULLEY["body_l"] - AL_PULLEY["hub_l"], p0)
    jB = (pul[0] - 0.5 - BRG_6801["B"], pul[0] - 0.5)
    return dict(hub_h=hub_h, jA=jA, jB=jB, pulley=pul, belt_c=p0 - AL_PULLEY["body_l"] / 2, end=jB[0] - 1.0)


V2 = _v2_stack()


def kx_hub_v2():
    """Printed KX74 hub with the flange-coupling pocket underneath (pocket depth = flange thickness) and 4 insert holes."""
    from .kx import KX74, hub_pip
    h = hub_pip(KX74)
    h -= Pos(0, 0, -V2["hub_h"]) * Cylinder(FLANGE["od"] / 2 + 0.3, FLANGE["t"], align=(Align.CENTER, Align.CENTER, Align.MIN))
    for k in range(4):
        a = math.pi / 2 * k + math.pi / 4
        h -= Pos(FLANGE["bolt_r"] * math.cos(a), FLANGE["bolt_r"] * math.sin(a), -V2["hub_h"]) * Cylinder(2.8, FLANGE["t"] + 5, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return h


def small_stage_v2(teeth: int = 20):
    """(name, part, colour) list in the stage frame. teeth: 20 (pitch, yaw, elbow, lift) or 18 (roll)."""
    MINZ = (Align.CENTER, Align.CENTER, Align.MIN)
    v = V2
    flange = Pos(0, 0, -v["hub_h"]) * Cylinder(FLANGE["od"] / 2, FLANGE["t"], align=MINZ)
    flange += Pos(0, 0, -v["hub_h"] - FLANGE["clamp_l"]) * Cylinder(FLANGE["clamp_od"] / 2, FLANGE["clamp_l"], align=MINZ)
    flange -= Pos(0, 0, -200) * Cylinder(SHAFT_D / 2, 400, align=MINZ)
    shaft_top = -v["hub_h"] - FLANGE["clamp_l"] + 2
    shaft = Pos(0, 0, v["end"]) * Cylinder(SHAFT_D / 2, shaft_top - v["end"], align=MINZ)
    brg = lambda j: Pos(0, 0, j[0]) * (Cylinder(BRG_6801["D"] / 2, BRG_6801["B"], align=MINZ) - Cylinder(BRG_6801["d"] / 2, BRG_6801["B"], align=MINZ))
    pd = teeth * PITCH / math.pi
    p0, p1 = v["pulley"][1], v["pulley"][0]
    pul = Pos(0, 0, p0 - AL_PULLEY["body_l"]) * (Cylinder(pd / 2 - PLD, AL_PULLEY["body_l"], align=MINZ))
    for z in (p0 - 1.5, p0 - AL_PULLEY["body_l"]):
        pul += Pos(0, 0, z) * Cylinder(AL_PULLEY["flange_od"] / 2, 1.5, align=MINZ)
    pul += Pos(0, 0, p1) * Cylinder(AL_PULLEY["hub_d"] / 2, AL_PULLEY["hub_l"], align=MINZ)
    pul -= Pos(0, 0, -200) * Cylinder(SHAFT_D / 2, 400, align=MINZ)
    return [("kx_hub_v2", kx_hub_v2(), "dark"), ("flange_coupling", flange, "steel"), ("shaft_12", shaft, "steel"),
            ("brg_A_6801", brg(v["jA"]), "steel"), ("brg_B_6801", brg(v["jB"]), "steel"), ("al_pulley", pul, "steel")]


def eccentric_bushing_v2() -> Part:
    """Eccentric bushing for a 6801 (Ø21 seat, e = 1 mm) in the same Ø38 wall bore: thinnest wall ≈ 7.5 mm (was ≈ 2 mm)."""
    e = ECC_V2
    r = Cylinder(e["od"] / 2, e["t"], align=(Align.CENTER, Align.CENTER, Align.MIN)) - Pos(e["e"], 0, 0) * Cylinder(BRG_6801["D"] / 2, e["t"], align=(Align.CENTER, Align.CENTER, Align.MIN))
    r += Pos(e["od"] / 2 + 3, 0, 0) * (Box(10, 8, 2.5, align=(Align.CENTER, Align.CENTER, Align.MIN)) - Pos(2, 0, 0) * Cylinder(1.7, 3, align=(Align.CENTER, Align.CENTER, Align.MIN)))
    return r


# Loop 40 (R14): split big pulleys. A printed pulley slides onto its Ø40 tube between the walls and drives it through a
# double-D (two flats, 36 mm across): the tube still passes the 61808/6708 bores, so it can be inserted from one side.
DD_AF = 36.0


def dd_prism(h: float, af: float = DD_AF, r: float = 20.0) -> Part:
    """Double-D section (Ø2r with two flats af apart, flats normal to X) extruded along Z from z = 0."""
    return Cylinder(r, h, align=(Align.CENTER, Align.CENTER, Align.MIN)) & Box(af, 2 * r + 2, h, align=(Align.CENTER, Align.CENTER, Align.MIN))


def dd_flats(z0: float, h: float, af: float = DD_AF, r: float = 20.0) -> Part:
    """What to subtract from a Ø2r tube (axis Z) to cut the two flats over z0 … z0 + h."""
    return Pos(0, 0, z0) * (Cylinder(r + 1, h, align=(Align.CENTER, Align.CENTER, Align.MIN)) - dd_prism(h, af, r))


def split_pulley(teeth: int, width: float = 16.0, flange: bool = False, af: float = DD_AF, web: bool = True) -> Part:
    """Printed HTD 5M pulley with a double-D bore (+0.4 mm), lightened to a 5 mm web when it is large."""
    p = pulley(teeth, width, flange=flange) - dd_prism(width + 10, af + 0.4, 20.2).translate((0, 0, -5))
    r_rim = teeth * PITCH / math.pi / 2 - PLD - DEPTH - 5.0
    if web and r_rim > 34:
        for z in (0.0, width / 2 + 2.5):
            p -= Pos(0, 0, z) * (Cylinder(r_rim, width / 2 - 2.5, align=(Align.CENTER, Align.CENTER, Align.MIN)) - Cylinder(28, width / 2 - 2.5, align=(Align.CENTER, Align.CENTER, Align.MIN)))
    return p


# ---------------------------------------------------------------------------------------------------------------------
# D24 (2026-11-27): small-pulley stage v3 for the bolted cartridge (cartridge_m.can_bolted). Between the walls it is the
# D13 v2 stack unchanged (Ø12 shaft, 2 × 6801 in eccentric bushings, aluminium HTD pulley); the KX74 hub, collar and
# flange coupling go, and an Oldham coupling (clamp hub on the shaft end) takes the motor's output.
# Stage frame: z = 0 is the cartridge flange mating plane = the cartridge-side face of wall A (6 mm, z −6 … 0), so V3 is
# V2 shifted up by the 23.7 mm the KX hub and coupling took above wall A. Callers keep their walls where they were.
V3_SHIFT = -(V2["jA"][1] + 1.0)                          # 23.7
V3 = {k: (tuple(x + V3_SHIFT for x in v) if isinstance(v, tuple) else v + V3_SHIFT) for k, v in V2.items() if k != "hub_h"}
V3["wall_a"] = (-6.0, 0.0)


def small_stage_v3(teeth: int = 20, motor: dict | None = None):
    """(name, part, colour) list in the stage frame (flange plane z = 0, cartridge +Z): the Oldham drive, Ø12 shaft,
    2 × 6801 and the aluminium pulley. The cartridge itself is bolted_cartridge(); the wall features are wall_a_cuts().
    motor: a cartridge_m.MOTORS entry (default the GIM6010-8; D20: the GIM8108-8 with its Ø50 Oldham)."""
    from . import cartridge_m as cm
    MINZ = (Align.CENTER, Align.CENTER, Align.MIN)
    v = V3
    lv = cm.bolted_levels(motor)
    shaft_top = lv["hub"][1] - cm.OLDHAM["tongue_h"] - 1.0
    shaft = Pos(0, 0, v["end"]) * Cylinder(SHAFT_D / 2, shaft_top - v["end"], align=MINZ)
    brg = lambda j: Pos(0, 0, j[0]) * (Cylinder(BRG_6801["D"] / 2, BRG_6801["B"], align=MINZ) - Cylinder(BRG_6801["d"] / 2, BRG_6801["B"], align=MINZ))
    pd = teeth * PITCH / math.pi
    p0, p1 = v["pulley"][1], v["pulley"][0]
    pul = Pos(0, 0, p0 - AL_PULLEY["body_l"]) * (Cylinder(pd / 2 - PLD, AL_PULLEY["body_l"], align=MINZ))
    for z in (p0 - 1.5, p0 - AL_PULLEY["body_l"]):
        pul += Pos(0, 0, z) * Cylinder(AL_PULLEY["flange_od"] / 2, 1.5, align=MINZ)
    pul += Pos(0, 0, p1) * Cylinder(AL_PULLEY["hub_d"] / 2, AL_PULLEY["hub_l"], align=MINZ)
    pul -= Pos(0, 0, -200) * Cylinder(SHAFT_D / 2, 400, align=MINZ)
    old = dict((n, p) for n, p, _ in cm.oldham_parts(motor))
    # names kept for callers (spacing / masses): "flange_coupling" = the shaft-side clamp hub, "kx_hub_v2" = the motor adapter
    return [("kx_hub_v2", old["drive_adapter"], "dark"), ("oldham_disc", old["oldham_disc"], "dark"),
            ("flange_coupling", old["oldham_hub"], "steel"), ("shaft_12", shaft, "steel"),
            ("brg_A_6801", brg(v["jA"]), "steel"), ("brg_B_6801", brg(v["jB"]), "steel"), ("al_pulley", pul, "steel")]


def bolted_cartridge(motor: dict | None = None, site: str = "default"):
    """(name, part, colour) of the bolted cartridge M in the stage frame: sleeve, rear cap, motor. site: the ear pattern
    (cartridge_m.EAR_SITES; "lift" on the lift, "default" elsewhere)."""
    from . import cartridge_m as cm
    return [("can", cm.can_bolted(motor, site), "white"), ("can_cap", cm.can_cap_bolted(motor), "white"), ("motor", cm.motor_bolted(motor), "dark")]


def bolted_stage(T, teeth: int = 20, prefix: str = "", motor: dict | None = None, site: str = "default"):
    """Cartridge + stage v3 mapped by T (stage frame → module frame), names prefixed. One call per joint."""
    return [(prefix + n, T * p, c) for n, p, c in bolted_cartridge(motor, site) + small_stage_v3(teeth, motor)]


def wall_a_cuts(motor: dict | None = None, site: str = "default") -> Part:
    """What to subtract from wall A (stage frame, wall z −6 … 0): the eccentric-bushing seat (Ø38.2), the counterbore for the
    cartridge's centring lip and, per flange ear (D24 v2, cartridge_m.EAR), an M4 clearance hole through the wall plus a
    hex pocket for the ISO 4032 nut on the pulley-side face (z −6 … −2.6: the screw comes from the cartridge side, so the
    nut and the wall are in compression).
    Neon #2 finding 2 (2026-11-27), per ear (cartridge_m.EAR_REVERSED / EAR_FEED):
      - reversed ear: the clearance hole only (head + washer on the pulley-side face, the nut is in the ear);
      - side-fed ear: instead of the pocket, a slot inside the wall (z −4.7 … −1.3, nut AF + 0.1 wide) from the screw
        axis to the wall's edge along the feed direction; the nut slides in from the edge and bears on the 1.3 mm skin
        between the slot and the ear (compression), so it is captive whichever way the wall faces."""
    from build123d import RegularPolygon, extrude
    from . import cartridge_m as cm
    MINZ = (Align.CENTER, Align.CENTER, Align.MIN)
    n, f = cm.nose_flange(motor, site)
    e = cm.EAR
    c = Pos(0, 0, -20) * Cylinder(ECC_V2["od"] / 2 + 0.1, 40, align=MINZ)
    c += Pos(0, 0, -n["lip"] - 0.1) * Cylinder(n["r_out"] + 0.1, 5, align=MINZ)
    hexp = extrude(RegularPolygon((e["nut_af"] + 0.1) / math.sqrt(3), 6), e["pocket"] + 0.01)
    for a in f["ang"]:
        t = math.radians(a)
        at = Pos(f["bolt_r"] * math.cos(t), f["bolt_r"] * math.sin(t), 0)
        if a in f["rev"]:
            c += at * Pos(0, 0, -6.5) * Cylinder(e["clr_d"] / 2, 7.0, align=MINZ)
        elif a in f["feed"]:
            c += at * Pos(0, 0, -6.5) * Cylinder(e["clr_d"] / 2, 7.0, align=MINZ)
            L0, L1 = e["nut_af"] / 2 + 0.05, 60.0
            c += at * Rot(0, 0, f["feed"][a]) * Pos((L1 - L0) / 2, 0, -6.0 + e["slot_skin"]) * Box(L1 + L0, e["nut_af"] + 0.1, 6.0 - 2 * e["slot_skin"], align=MINZ)
        else:
            c += at * Pos(0, 0, -10) * Cylinder(e["clr_d"] / 2, 20, align=MINZ)
            c += at * Pos(0, 0, -6.01) * Rot(0, 0, a) * hexp
    return c


def wall_b_cuts(motor: dict | None = None, site: str = "default", z_far: float = -80.0, r: float | None = None) -> Part:
    """Neon #2 finding 2: what to subtract from the stage's OPPOSITE wall (stage frame; anything between z_far and wall A's
    pulley face, z −6.5) for each reversed flange ear (cartridge_m.EAR_REVERSED): a Ø10 hole on the screw axis, so the
    M4 × 16 + washer (Ø9) go in and out through it and a Ø6 driver reaches the head on wall A's pulley face. Callers:
    shoulder.py outer wall (pitch4: 160° / −120°): `w -= _ST * wall_b_cuts(MOT, EAR_SITE)` in _wall(..., inner=False)."""
    from . import cartridge_m as cm
    _, f = cm.nose_flange(motor, site)
    rr = (r or cm.EAR["rev_hole_d"] / 2)
    out = None
    for a in f["rev"]:
        t = math.radians(a)
        h = Pos(f["bolt_r"] * math.cos(t), f["bolt_r"] * math.sin(t), z_far) * Cylinder(rr, -6.5 - z_far, align=(Align.CENTER, Align.CENTER, Align.MIN))
        out = h if out is None else out + h
    return out


def ear_boss(motor: dict | None = None, site: str = "default", r: float = 7.5, t: float = 6.0) -> Part:
    """Wall-A material each ear needs (stage frame, z −t … 0): a disc of radius r round each ear screw (nut pocket 4.0 across
    corners + ≥ 3 mm), for walls that do not already reach r 55 (elbow cassette plate A)."""
    from . import cartridge_m as cm
    _, f = cm.nose_flange(motor, site)
    out = None
    for a in f["ang"]:
        t_ = math.radians(a)
        d = Pos(f["bolt_r"] * math.cos(t_), f["bolt_r"] * math.sin(t_), -t) * Cylinder(r, t, align=(Align.CENTER, Align.CENTER, Align.MIN))
        out = d if out is None else out + d
    return out
