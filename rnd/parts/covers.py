"""D23 covers v1 (owner, 2026-11-27: "exposed motors could pinch someone; make it consumer-facing and safe"; R17).

Printable covers that close the drive parts to a Ø12 mm finger probe and keep the gaps at the joints either ≤ 8 mm or
≥ 25 mm (checked by rnd/pinch.py over the joint sweeps). Every rigid piece is PETG (soft-touch finish), wall ≥ WALL,
≤ 256 mm on the P2S (assembly_check printability lists them via printed()), split where it has to go round a link,
and fixed with M3 screws into heat-set inserts (INSERT) or with snap lugs. Soft pieces (names with gaiter / boot /
bellows) are TPU 95A prints or sewn fabric: they close an opening across a joint and give under a finger.

Each cover is built in the frame of the body it rides on, so robot.build(covers=True) moves it with its joint. Sizes
come from the CAD of the parts they cover (bounding boxes and the module constants), so they follow arm/roll edits;
re-run rnd/pinch.py after any change.

Status (loop of 2026-11-27): elbow + forearm, wrist boot, lift (torso collars, spine pod, belt duct, mast cap; the
closed lift cover is upper.lift_cover, the head's TPU neck boot is upper.neck_boot) and base (belly pan, wheel-well
liners) are printable. The shoulder dome and
yaw sleeve (pitch_frame / roll_frame) are still the render-only concept (see the note there). The elbow dome's 4 M3
need heat-set inserts in the housing wall (rnd/parts/arm.py, next arm edit).

Wave 5 (shoulder roll + neck, R17): the inline-roll yaw hood now runs over the yaw can's front 10 mm (it was open there
and the probe reached the can's pigtail window and the motor through the hood's front corners). Compliant (TPU) covers
are judged by rnd/pinch.py's rule (compliant() below): the shoulder's inboard wedge (yaw block vs ferrule, ≈ 2 mm at
roll −15°, ≈ 135 mm at 100°) cannot be closed by any boot within ±40 % and the D22 neck boot's floor would have to
follow the riser hub ≈ 84 mm; both need a geometry or range decision (see compliant()).

Neck CAD pass (R17, 2026-11-27): the head's back is now round the tilt axis and the neck slot is closed by a rigid saddle
on the neck bar at a constant 4 mm (upper.py, neck v2), so the neck boot and its two compliant() rows are gone.

R17 W18 (2026-11-28): the elbow scissor is closed by geometry (arm.ELBOW_GAP: the forearm 25.5 mm off the housing), the
yaw row by a round skirt under the yaw plate (roll.yaw_skirt), two shoulder–torso rows by roll.FRONT_FRAME / roll.BAND_IN;
the hood's inboard wedge is still open (see the roll notes above roll_frame()).

R17 W20 (2026-11-28): the yaw can's rear-cap lugs at ±125° (cartridge_m.CAP_SITES) put the hood's inboard wall at y 20.5
(cover_yaw × torso_shell closed); PIP_WEDGE_GUARD=skirt adds the TPU comb skirt over the hood's upper inboard wedge
(wedge_skirt), reported by rnd/pinch.py as soft-guarded, not as a rigid pass.

R17 pass (2026-11-28): the elbow bay dome is redrawn round the GIM6010 can (its flange ears stood outside the concept box)
and the wrist boot is a round, closed boot (the can flange faced the hand): no exposed motor can left. The roll hard stop
moved inside the front collar (roll.FRONT_STOP) and the yaw hood's front is squared off the now flat rear plate.
"""
from __future__ import annotations

import math

from build123d import Align, Axis, Box, Cylinder, Pos, Rot

from .soft import soft_box, soft_shell

WALL = 2.2                       # rigid cover wall (≥ 2 mm rule)
TPU_WALL = 2.0                   # soft gaiters / boots
INSERT = dict(d=4.0, h=5.7)      # M3 heat-set insert hole (short M3 insert: Ø4.0 × 5.7 hole is the usual maker figure, ESTIMATE: check the datasheet)
M3_CLEAR = 3.4
SPLIT_GAP = 0.3                  # parting gap between cover halves
MIN = (Align.CENTER, Align.CENTER, Align.MIN)


# ---------------------------------------------------------------------------------------------- helpers
def _bb(p):
    b = p.bounding_box()
    return b.min.X, b.max.X, b.min.Y, b.max.Y, b.min.Z, b.max.Z


def _main(part):
    """The largest solid of a boolean result (drops slivers left between the cover and what it was cut round)."""
    sols = part.solids()
    return max(sols, key=lambda q: q.volume) if len(sols) > 1 else part


def _boss(x, y, z, axis="z", h=8.0, insert=True):
    """Ø9 boss with an M3 insert hole (or a clearance hole) along `axis`."""
    r = Rot(0, 90, 0) if axis == "x" else (Rot(90, 0, 0) if axis == "y" else Rot(0, 0, 0))
    hole = INSERT["d"] / 2 if insert else M3_CLEAR / 2
    return Pos(x, y, z) * r * (Cylinder(4.5, h) - Cylinder(hole, h))


def _halves(part, axis, at, gap=SPLIT_GAP):
    """Split a cover in two along the plane axis = at (parting gap `gap`)."""
    x0, x1, y0, y1, z0, z1 = _bb(part)
    big = 2 * max(x1 - x0, y1 - y0, z1 - z0) + 50
    if axis == "x":
        lo = Pos(at - gap / 2 - big / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(big, big, big)
        hi = Pos(at + gap / 2 + big / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(big, big, big)
    elif axis == "y":
        lo = Pos((x0 + x1) / 2, at - gap / 2 - big / 2, (z0 + z1) / 2) * Box(big, big, big)
        hi = Pos((x0 + x1) / 2, at + gap / 2 + big / 2, (z0 + z1) / 2) * Box(big, big, big)
    else:
        lo = Pos((x0 + x1) / 2, (y0 + y1) / 2, at - gap / 2 - big / 2) * Box(big, big, big)
        hi = Pos((x0 + x1) / 2, (y0 + y1) / 2, at + gap / 2 + big / 2) * Box(big, big, big)
    return part & lo, part & hi


def _split_bosses(axis, at, pts, h=8.0):
    """Insert bosses on the low side and clearance bosses on the high side of a split plane, at `pts` (the two
    coordinates in the plane)."""
    a, b = None, None
    for u, v in pts:
        if axis == "x":
            lo, hi = _boss(at - h / 2 - SPLIT_GAP / 2, u, v, "x", h, True), _boss(at + h / 2 + SPLIT_GAP / 2, u, v, "x", h, False)
        elif axis == "y":
            lo, hi = _boss(u, at - h / 2 - SPLIT_GAP / 2, v, "y", h, True), _boss(u, at + h / 2 + SPLIT_GAP / 2, v, "y", h, False)
        else:
            lo, hi = _boss(u, v, at - h / 2 - SPLIT_GAP / 2, "z", h, True), _boss(u, v, at + h / 2 + SPLIT_GAP / 2, "z", h, False)
        a = lo if a is None else a + lo
        b = hi if b is None else b + hi
    return a, b


def _split_cover(shell, axis, at, pts, h=8.0):
    """Two printable halves of `shell`, each with its half of the M3 bosses."""
    lo, hi = _halves(shell, axis, at)
    bl, bh = _split_bosses(axis, at, pts, h)
    return lo + bl, hi + bh


# ---------------------------------------------------------------------------------------------- shoulder (roll joint)
# Still the render-only concept (not in printed()). Over roll −15…100° the yaw block swings out to the horizontal
# and sweeps through most of the space this dome occupies: a static dome cannot enclose the roll joint here (a
# swept-clearance cut leaves ≈ 3 cm³ of it). Needs a smaller dome over the roll cartridge + a TPU boot across the
# joint, best drawn on the D24 inline-roll layout that is replacing this stage.
def pitch_frame():
    """Concept: rides on the pitch body (static w.r.t. roll): a dome over the roll link, the roll cartridge, the 80T
    pulley and its belt, down to just above the roll axis; open underneath where the yaw block swings out.
    Belt roll only: on the D24 inline roll (default) there is no roll belt, and this dome sat across the yaw cartridge
    (≈ 6 cm³ of overlap). Inline roll: the static front collar on the motor face (roll_front_collar, R17 2026-11-28)."""
    from . import roll
    if roll.INLINE:
        return [("cover_roll_front", roll_front_collar(), "white")]
    x0, x1, y0, y1, z0, z1 = -122.0, 62.0, 4.0, roll.ROLL_Y + 72, -30.0, roll.ROLL_C + 78
    d = Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * soft_shell(x1 - x0, y1 - y0, z1 - z0, 46, 34, 2.2)
    d -= Pos((x0 + x1) / 2, (y0 + y1) / 2, z0) * Box(x1 - x0 - 6, y1 - y0 - 6, 8)
    d -= Pos(0, roll.ROLL_Y, -30) * Box(x1 - x0 + 10, 120, 70)                         # the arm exits below the roll axis
    return [("cover_shoulder", d, "white")]


# R17 (2026-11-28): the inline roll's output face. Between the GIM8108's body front face (x 25, r 48) and the rotating
# Ø76 output adapter (x 41.5…48.5) and clevis boss (r 42, x 48.5…62.5) lies an open pocket round the motor's Ø60 / Ø42
# steps; the adapter's inboard face sat 6.8…8.3 mm from the steps (roll_outflange × roll_gim_housing failed on the 8 mm
# line) and the boss's rim faced the roll link's frame across 9–14 mm (yaw_block × roll_link). A static PETG collar on the
# body face, a surface of revolution about the roll axis, fills the pocket: its face runs 2 mm in front of the adapter
# (r 30.3…40) and its rim (r 40…46) runs on to 2 mm short of the boss, 2 mm outside the adapter's rim. Every rotating
# surface there then has static within 2 mm at every roll angle, the same at every angle.
# R17 (2026-11-28): the roll hard stop moved in here (roll.FRONT_STOP): the collar now stands on the 3 mm 7075 stop ring
# (x 25…28) and is hollow inside r 38 (the channel where the adapter's two dogs run between the ring's two blocks, whose
# tops make the collar's face there). Its outer faces are unchanged. Held by 2 × M3 × 10 (ISO 4762) radial through its
# outer wall (counterbored flush) into the blocks: the W14b 4 × M4 × 25 into the front-face holes were too long for
# the 3.5 mm-deep threads (7.5 mm past a 17.5 mm grip) and those holes now take the ring's countersunk M4.
FRONT_COLLAR = dict(bore=30.3, r_face=40.0, r_out=46.0, gap=2.0, r_chan=38.0, cb=(6.0, 3.3))


def roll_front_collar():
    """Static collar on the inline roll motor's front face (roll-module frame; covers.pitch_frame)."""
    from . import roll
    c, f = FRONT_COLLAR, roll.FRONT_STOP
    xb = roll.X_MOT[0] + roll.MOT36["body_h"] + f["base_t"]          # on the stop ring (body front face 25 + 3)
    xf = roll.X_ADAPT[0] - c["gap"]                                  # face, 2 mm in front of the adapter
    xr = roll.X_ARM_F[0] - c["gap"]                                  # rim end, 2 mm short of the clevis boss
    ax = lambda x, r, L: Pos(x, roll.ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(r, L, align=(Align.CENTER, Align.CENTER, Align.MIN))
    p = ax(xb, c["r_out"], xf - xb) + ax(xf, c["r_out"], xr - xf) - ax(xf, c["r_face"], xr - xf + 1)
    p -= ax(xb - 1, c["r_chan"], xr - xb + 2)                       # the stop channel + blocks (r ≤ 37.5) inside
    m3 = f["collar_m3"]
    for a0, a1 in roll.front_stop_angles()["block"]:
        am = (a0 + a1) / 2
        rot = Rot(am + 180.0, 0, 0)
        p -= Pos(m3["x"], *roll._ang(am, c["r_chan"] - 1)) * rot * Cylinder(1.7, c["r_out"] - c["r_chan"] + 2, align=(Align.CENTER, Align.CENTER, Align.MIN))
        p -= Pos(m3["x"], *roll._ang(am, c["r_out"] - c["cb"][1])) * rot * Cylinder(c["cb"][0] / 2, c["cb"][1] + 1, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return p


_HOOD = []
_FUZZ = 1e-5          # mm
HOOD_FRONT = 3.5      # R17 (2026-11-28): the hood's front face this far from the (now flat) rear plate (x −29.5; was the box
                      # end 5.2 mm past the can, x −33, cut back to x −35.5 round the W13 stop bushes' sweep). Tried 6 and 4:
                      # the same two rows fail, with higher maxima (rnd/pinch.py)


def _cut_main(h, q):
    """h − q as a fuzzy boolean (1e-5 mm), keeping only the largest solid. The ≈ 820 swept cuts of the yaw hood used to run
    as plain `h -= q`: one cut near roll −12° left an edge tolerance of ≈ 1e140, the ≈ 200 slivers were carried into
    every later cut, and the cuts after roll 0° doubled in time and memory until rnd.assembly_check ran out of memory
    (12 GB, D26). Fuzzy + main solid: ≈ 2 min, 0.6 GB, tolerance ≤ 1e-4, one solid (the slivers were dropped at the end
    anyway, _main)."""
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
    op = BRepAlgoAPI_Cut()
    op.SetFuzzyValue(_FUZZ)
    r = h._bool_op((h,), (q,), op)
    sols = r.solids()
    return max(sols, key=lambda x: x.volume) if len(sols) > 1 else r
_GROW = [(0, 0, 0)] + [tuple(s * 1.5 * (i == k) for i in range(3)) for k in range(3) for s in (1, -1)]   # ≈ 1.5 mm growth


def _hood_box():
    """(x0, x1, y0, y1, z0, z1) of the yaw hood's outer box (roll-module frame): WALL + 3 mm round the yaw can's bounding box
    (R17 2026-11-28: with the can's rear-cap lugs at ±125°, cartridge_m.CAP_SITES, its inboard side is the sleeve, so
    y0 = 69 − 43.3 − 5.2 = 20.5, 25.5 off the torso), its front squared HOOD_FRONT off the roll motor's rear plate."""
    if not _HOOD_BOX:
        from . import roll
        can = [p for n, p, _ in roll.module() if n == "yaw_can"][0].bounding_box()
        t, g = WALL, 3.0
        _HOOD_BOX.append((can.min.X - g - t, roll.X_PLATE[0] - HOOD_FRONT, can.min.Y - g - t, can.max.Y + g + t, roll.ZF + 0.5, can.max.Z + g + t))
    return _HOOD_BOX[0]


_HOOD_BOX = []


def _yaw_hood():
    """D24 inline roll (2026-11-27): rides on the roll output. A 2.2 mm hood 3 mm round the bolted yaw cartridge (can +
    rear cap), standing on the yaw block's top plate (the yaw belt is already inside the yaw band), its front squared
    HOOD_FRONT off the rear plate (R17). Cut clear of the static roll link, motor housing, ferrule and spring ring swept
    over the roll range (3° steps, ±1.5 mm on each axis), and 1.5 mm off the roll output's rear arm. Wave 5: it no longer stops at x −46.5 (that left the can's front 10 mm and the
    hood's front corners open); it runs over the can's front to its own box end (5.2 mm past the can), cut only by those
    swept parts. One PETG print (≈ 99 × 109 × 71 mm, in printed()); fixing still to draw: 4 × M3 tabs at its foot into
    heat-set inserts in the yaw block's top plate (inserts: next rnd/parts/roll.py edit). The inboard wedge between the yaw block
    and the ferrule stays open (see compliant())."""
    if not _HOOD:
        from . import roll
        from .. import swept
        mv, st, o, _, (lo, hi) = swept.joints()["roll"]
        t = WALL
        x0, x1, y0, y1, z0, z1 = _hood_box()
        h = Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * soft_shell(x1 - x0, y1 - y0, z1 - z0, 14, 8, t)
        h -= Pos((x0 + x1) / 2, (y0 + y1) / 2, z0) * Box(x1 - x0 - 2 * t, y1 - y0 - 2 * t, 2 * t + 1)
        R = lambda a: Pos(0, o[1], o[2]) * Rot(a, 0, 0) * Pos(0, -o[1], -o[2])
        hx = h.bounding_box().max.X
        for a in range(int(lo) - 3, int(hi) + 4, 3):
            for _, p in st:
                for d in _GROW:
                    q = Pos(*d) * (R(-a) * p)
                    if q.bounding_box().min.X <= hx:
                        h = _cut_main(h, q)
        for n, p in mv:
            if n == "roll_arm_rear":
                for d in _GROW:
                    h = _cut_main(h, Pos(*d) * p)
        # R17 (2026-11-27, wave 5): the hood used to stop at x −46.5 (1.5 mm behind the roll link), which left its
        # front corners (box round a round can) open and the probe reached the can's pigtail window (12 × 16) and the
        # motor through them. It now runs on over the can's front 10 mm to 5.2 mm past the can (its own box end),
        # cut only by the static parts swept over the roll range (+1.5 mm) and the rear clevis arm (+1.5 mm).
        _HOOD.append(_main(h))
    return _HOOD[0]



# PIP_WEDGE_GUARD=skirt (R17, 2026-11-28; owner call pending against ROLL_Y + 23–30 mm). The hood's upper inboard wedge:
# its inboard face (y 20.5, x ≤ −29.5) and the inboard part of its top beside the roll link's ferrule boss (a Ø90 disc
# about the pitch axis, outboard face y 12, x ≥ −45), the link above it and the ferrule rim. These are planes parallel to
# the roll axis, so the gap at each point of the hood runs from ≈ 1–9 mm to 6–38 mm over −15…100° (no rigid fix, see the
# notes below). The soft option keeps a finger off the hood there instead of holding a gap: a TPU 95A comb skirt bonded
# and clipped to the hood over x WEDGE_SKIRT["x"] (inboard face, the rounded top-inboard edge and a top strip out to
# y0 + top_w), fins in planes normal to the roll axis (each fin turns in its own plane; where the boss, the link or the
# ferrule rim comes into a fin it is pushed sideways and lays over on its neighbour, it does not have to compress),
# WEDGE_SKIRT["pitch"] apart (the slots between fins ≤ 8 mm, closed to a finger), fin_h tall off a backing sheet. The fins
# are cut 1.5 mm clear of every static part at the reference pose (roll 0°) and of the rear clevis arm, and taken out
# (± 2 mm) wherever a static part comes closer to the hood than the folded stack at some angle, so the TPU is never the
# thing crushed. Inboard, fin_h ends the fins at y 12 (≥ 5.6 mm off the torso at roll −15°). It is not a pinch surface and
# not a rigid guard: rnd/pinch.py takes it out for the gate and reports the rows it closes as soft-guarded, with the force
# limit (D25 / R17) as the second barrier, and judges it by its rule (wedge_skirt_rule, skirt_roots, skirt_stack).
WEDGE_SKIRT = dict(wall=1.6, pitch=6.0, x=(-100.0, -31.0), fin_h=8.5, top_w=30.0, clear=1.5, sheet=1.6,
                   E_mpa=26.0)   # TPU 95A flexural modulus ≈ 26 MPa (typical filament datasheets: 20–40; ESTIMATE)
_SKIRT = []


def _skirt_fin_x():
    """x of each fin's +X face (the front fin first)."""
    k = WEDGE_SKIRT
    out, x = [], k["x"][1]
    while x - k["wall"] >= k["x"][0] - 1e-6:
        out.append(x)
        x -= k["pitch"]
    return out


def wedge_skirt():
    """PIP_WEDGE_GUARD=skirt: the TPU 95A comb skirt on the yaw hood's inboard face and top edge (roll-module frame, rides
    on the roll output). One TPU print, ≈ 69 × 39 × 83 mm."""
    if not _SKIRT:
        from .. import swept
        mv, st, o, _, _ = swept.joints()["roll"]
        k = WEDGE_SKIRT
        x0, x1, y0, y1, z0, z1 = _hood_box()
        xc, yc, zc = (x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2
        grow = lambda e: Pos(xc, yc, zc) * soft_box(x1 - x0 + 2 * e, y1 - y0 + 2 * e, z1 - z0 + 2 * e, 14 + e, 8 + e)
        env = grow(0.0)                                                                          # the hood's outer solid
        H, sh = k["fin_h"], k["sheet"]
        ya, yb, za, zb = y0 - H - 1, y0 + k["top_w"], z0, z1 + H + 1
        region = Pos((k["x"][0] + k["x"][1]) / 2, (ya + yb) / 2, (za + zb) / 2) * Box(k["x"][1] - k["x"][0], yb - ya, zb - za)
        sheet = (grow(sh) - env) & region
        shell = (grow(H) - env) & region
        fins = None
        for x in _skirt_fin_x():
            f = Pos(x - k["wall"] / 2, (ya + yb) / 2, (za + zb) / 2) * Box(k["wall"], yb - ya, zb - za)
            fins = f if fins is None else fins + f
        sk = (shell & fins) + sheet
        g = k["clear"]
        cut = [p for _, p in st] + [p for n, p in mv if n == "roll_arm_rear"]
        for p in cut:
            for d in [(0, 0, 0)] + [tuple(sg * g * (i == j) for i in range(3)) for j in range(3) for sg in (1, -1)]:
                sk = _cut_main(sk, Pos(*d) * p)
        for c, size in _skirt_crush_boxes(st, o):                                               # no TPU where it would be crushed
            sk = _cut_main(sk, Pos(*c) * Box(*size))
        _SKIRT.append(_main(sk))
    return _SKIRT[0]


def skirt_roots():
    """[(x, y, z, (nx, ny, nz), kind)] root points of the comb skirt on the hood surface (roll-module frame) with the
    hood's outward normal there: kind "fin" at each fin's mid-plane, "sheet" half a pitch behind it; every ≈ 1 mm along the
    inboard face (−Y), the rounded top-inboard edge (r 8) and the top strip (+Z)."""
    k = WEDGE_SKIRT
    x0, x1, y0, y1, z0, z1 = _hood_box()
    re = 8.0
    path = [(y0, float(z), (0.0, -1.0, 0.0)) for z in range(int(math.ceil(z0)), int(z1 - re))]
    for a in range(0, 91, 8):
        t = math.radians(a)
        path.append((y0 + re - re * math.cos(t), z1 - re + re * math.sin(t), (0.0, -math.cos(t), math.sin(t))))
    path += [(float(y), z1, (0.0, 0.0, 1.0)) for y in range(int(y0 + re) + 1, int(y0 + k["top_w"]) + 1)]
    out = []
    for x in _skirt_fin_x():
        for xx, kind in ((x - k["wall"] / 2, "fin"), (x - k["wall"] - (k["pitch"] - k["wall"]) / 2, "sheet")):
            dy = 0.0 if xx <= x1 - 14 else 14 - math.sqrt(max(0.0, 14 ** 2 - (xx - (x1 - 14)) ** 2))   # the hood's front-inboard corner (r 14)
            out += [(xx, y + (dy if n[1] < -0.5 else 0.0), z, n, kind) for y, z, n in path]
    return out


def skirt_stack(kind="fin"):
    """Thickness (mm) the skirt needs between the hood and a static part where it is pushed flat: the backing sheet plus,
    at a fin, the fins laid over each other (fin height / pitch layers, ≥ 1)."""
    k = WEDGE_SKIRT
    if kind == "sheet":
        return k["sheet"] + 1.0
    return k["sheet"] + k["wall"] * max(1.0, k["fin_h"] / k["pitch"])


def _skirt_field(st, o, step=2.5):
    """Nearest static distance (mm) from each skirt root point over the roll range: (roots, array [root, angle], angles)."""
    import numpy as np
    import trimesh
    from scipy.spatial import cKDTree
    from ..twin import JR
    S = []
    for _, p in st:
        v, t = p.tessellate(0.4, 0.3)
        m = trimesh.Trimesh(np.array([(q.X, q.Y, q.Z) for q in v]), np.array(t), process=False)
        S.append(trimesh.sample.sample_surface(m, max(40, int(m.area * 1.0)), seed=11)[0])
    tree = cKDTree(np.concatenate(S))
    R = skirt_roots()
    P = np.array([r[:3] for r in R])
    c = np.asarray(o, float)
    ang = np.arange(JR["roll"][0], JR["roll"][1] + 1e-6, step)
    G = np.empty((len(P), len(ang)))
    for j, a in enumerate(ang):
        t = math.radians(a)
        M = np.array([[1, 0, 0], [0, math.cos(t), -math.sin(t)], [0, math.sin(t), math.cos(t)]])
        G[:, j] = tree.query((P - c) @ M.T + c)[0]
    return R, G, ang


def _skirt_crush_boxes(st, o):
    """[(centre, size)] boxes cut out of the skirt: a fin (or the sheet between two fins) wherever a static part comes
    closer to the hood than skirt_stack() at some roll angle (1° steps), ± 2 mm along the surface, through the fin's height
    (from the sheet's face up when the sheet alone still fits there)."""
    k = WEDGE_SKIRT
    R, G, _ = _skirt_field(st, o, step=1.0)
    gmin = G.min(1)
    boxes, prev = [], None
    for i, ((x, y, z, n, kind), g) in enumerate(zip(R, gmin)):
        if g >= skirt_stack(kind):
            continue
        h = (k["wall"] + 0.6 if kind == "fin" else k["pitch"] - k["wall"]) / 2
        L = k["fin_h"] + 2.0
        s0 = k["sheet"] + 0.05 if kind == "fin" and g >= skirt_stack("sheet") else -2.0   # keep the sheet under a fin if it fits
        ys, zs = (y + n[1] * s0, y + n[1] * L), (z + n[2] * s0, z + n[2] * L)
        b = [x - h, x + h, min(ys) - 2.0 * (abs(n[1]) < 0.9), max(ys) + 2.0 * (abs(n[1]) < 0.9), min(zs) - 2.0 * (abs(n[2]) < 0.9), max(zs) + 2.0 * (abs(n[2]) < 0.9)]
        if prev is not None and prev[0] == i - 1 and prev[1] == (x, kind, s0 > 0):            # merge along the surface
            c = boxes[-1]
            boxes[-1] = [min(c[0], b[0]), max(c[1], b[1]), min(c[2], b[2]), max(c[3], b[3]), min(c[4], b[4]), max(c[5], b[5])]
        else:
            boxes.append(b)
        prev = (i, (x, kind, s0 > 0))
    return [(((b[0] + b[1]) / 2, (b[2] + b[3]) / 2, (b[4] + b[5]) / 2), (b[1] - b[0], b[3] - b[2], b[5] - b[4])) for b in boxes]


def wedge_skirt_rule():
    """Declared for rnd/pinch.py (the soft-guard rule there): the skirt's part-name suffix, joint, wall, fin pitch, fin
    height and the fin bending stiffness input."""
    k = WEDGE_SKIRT
    return dict(part="cover_yaw_tpu_skirt", joint="roll", wall=k["wall"], pitch=k["pitch"], fin_h=k["fin_h"], E_mpa=k["E_mpa"])


# Roll nips (R17; 2026-11-28 pass, rnd/pinch.py):
# - Front, closed: roll_outflange × roll_gim_housing and yaw_block × roll_gim_housing by the static collar on the motor
#   face (roll_front_collar, above); yaw_block × roll_link and yaw_band × roll_link by raising the roll link's frame and
#   ferrule boss in front of the side block (roll.FRONT_FRAME) and turning its spring bar up (roll.SPR_BAR_ANG). The
#   lesson: a rotating cell passes only if static stays within 8 mm of it at every angle (a surface of revolution round
#   it) or never comes within 25 mm; a flat static wall beside a rotating round boss fails both ways.
# - Rear: cover_yaw × roll_gim_housing closed (R17, 2026-11-28). Root cause was the two W13 hard-stop bushes 8 mm proud of
#   the rear plate (the only non-round static feature in front of the hood; every hood cell within 25 mm saw them come
#   and go, and the hood had to be cut 9.5 mm back from the plate round their sweep). The stop moved inside the front
#   collar (roll.FRONT_STOP), the rear plate is flat and the hood's front is squared 3.5 mm off it (HOOD_FRONT): the row
#   now holds 3.5 mm at every angle. (A round-fronted hood, tried earlier, was worse: 10–18 mm walls.)
# - Rear, still open: cover_yaw × roll_link and cover_yaw × ferrule_m (people near too). The row minima shown in pinch.md
#   are the worst cell only: per cell (W18 diagnostic, 2026-11-28) the hood's inboard wall (y 14.3, x −131 … −29.5) has
#   cells running 1–2 → 11–23 mm and 7 → 26 mm over the roll range, not ones just over the 8 mm line. Cause: the static
#   faces within 25 mm of it (the ferrule's back face and rim, the boss, the frame) are planes parallel to the roll axis,
#   and a point that turns about the axis changes its distance to such a plane at every angle; only static faces normal
#   to the axis (the rear plate, the side block's −X face: the hood front's 3.5 mm row holds) or round about it keep a gap.
#   No rigid guard fixes it: a static guard round about the roll axis has to be a full ring (a sector has ends, and the
#   hood's sweep, 48 … 312° of the output frame plus the 115° roll range, covers every angle), and a full ring can only
#   live inside r 52 (the yaw block's top plate sweeps there), while the hood and the ferrule share the band r 51 … 73 in
#   x −45.5 … −29.5. A box study (the hood moved outboard by dy, gap to the ferrule + boss over −15 … 100°): dy 10 / 20 / 30
#   mm gives 4.9 / 12.1 / 18.2 mm (18.2 at +100°, where the hood's top passes over the roll link's top), so even a yaw
#   cartridge ≈ 30 mm outboard (≈ 21° about the yaw axis on the same 315-5M belt) does not reach 25 on its own. Options
#   for an owner call: ROLL_Y + ≈ 23–30 mm, or a soft (TPU) skirt plus the D23 (5) force limit. W20 (2026-11-28): the
#   skirt is built behind PIP_WEDGE_GUARD=skirt (wedge_skirt above; rnd/pinch.py item 7 lists both rows as soft-guarded,
#   the gate still counts them). ROLL_Y estimate (scratch study, hood + roll output + the link's far side moved out, the
#   ferrule and the link's boss kept; no CAD rebuild): + 23 mm takes the hood > 25 mm off the ferrule (that row closes) but
#   leaves cover_yaw × roll_link at 9.3 … 11.2 (the link's rear corner) and cells at 14 … 16 (near-shear, 1.2–1.9 mm swing);
#   + 30 mm: 3.8 … 10.2 at the link's lower rear corner. Neither closes the link row without reshaping the link; R5 door stow
#   W(45°) 560 → ≈ 577 (+ 23) / 583 (+ 30) mm by a twin re-search (≤ 570 allowed), W(0°) floor 644 → ≈ 690 / 704.
# - Inboard wedge (yaw block / band × ferrule, 2.6 / 2.7 mm at −15°): arm part against arm part, closed with people near
#   by the D25 soft limit (≥ +5°). Full-range fixes: a roll hard stop at ≥ +5° (R16 across-body 71 → 49 %) or ROLL_Y + 23
#   mm.
# - Shoulder–torso (pitch, W18 2026-11-28): roll_link × torso_shell closed by the front frame moved outboard (roll.FRONT_FRAME,
#   26 mm off the torso wall), yaw_band × torso_shell by the band's inboard face (roll.BAND_IN, 25.5 mm). W20 (2026-11-28):
#   cover_yaw × torso_shell (19.3 … 27.8) closed: the yaw can's rear-cap lugs turned from ±90° to ±125°
#   (cartridge_m.CAP_SITES["yaw"]), so the can's inboard side is the r 43.3 sleeve and the hood's inboard wall is at y 20.5,
#   25.5 off the torso (_hood_box). PIP_TORSO_RECESS was not needed. Side effect: the hood face now stands 8.5 mm (was 2.3)
#   off the roll link's ferrule boss at roll 0 (the wedge rows above: cover_yaw × ferrule_m 1.9 … 9.4 → 5.8 … 8.3,
#   cover_yaw × roll_link 0.8 … 9.1 → 3.1 … 19.1).
def roll_frame():
    """Concept: rides on the roll output: a sleeve round the whole yaw block (both plates, the belt, the yaw
    cartridge, its KX collar and torque fork), open at the top (the clevis arms) and the bottom (the yaw flange).
    Inline roll (default): the yaw-cartridge hood (_yaw_hood); the old sleeve's top followed ZF + ZC and it engulfed the
    roll link and motor (≈ 30 cm³ of overlap)."""
    from . import roll
    if roll.INLINE:
        from ..params import WEDGE_GUARD
        return [("cover_yaw", _yaw_hood(), "white")] + ([("cover_yaw_tpu_skirt", wedge_skirt(), "dark")] if WEDGE_GUARD == "skirt" else [])
    z0, z1 = roll.P2[0] - 4, roll.ZF + roll.ZC + 14
    c = Pos(-38, roll.ROLL_Y, (z0 + z1) / 2) * soft_shell(198, 122, z1 - z0, 46, 18, 2.2)
    c -= Pos(-38, roll.ROLL_Y, z0) * Box(194, 118, 6)
    c -= Pos(0, roll.ROLL_Y, z1) * Box(80, 70, 8)                                       # clevis arms rise through the top
    return [("cover_yaw", c, "white")]


# ---------------------------------------------------------------------------------------------- elbow + forearm
# R17 (2026-11-28, rnd/pinch.py: arm_e_can 343 mm², arm_e_can_cap 131 mm² exposed). The elbow bay dome was still the
# loop-23 concept box (104 × 40 × 104, soft corners r 19.6): it ran 0.3 mm³ into the can, the encoder bracket, the elbow
# shaft and the housing, and near the housing face its rounded corners pulled in to |x| ≈ 42 while the GIM6010 can's
# flange ears (r 49.7, horizontal) stand out to |x| 49.5, 0.5 mm wider than the housing: the probe reached the ears and
# the rear cap's rim through the four corners. Now a cylinder about the cartridge axis, 1.8 mm round the ears, closed
# 4 mm behind the rear cap, running on 14 mm past the housing's −Y face as a skirt beside its ±X faces (2.5 mm off them),
# cut by the housing envelope (+0.3), the upper bone itself, the encoder bracket and board (+1) and the elbow shaft's
# Ø44 end: the openings left are ≤ 2.5 mm. 4 × M3 bosses at r 47.6 between the ears (±45°, ±135°, where the can is r 43.3);
# their screws go into heat-set inserts in the housing's −Y wall, as the concept's (still to draw in arm.py).
ELBOW_DOME = dict(clear=1.8, end_gap=4.0, skirt=14.0, boss_r=47.6, boss_d=7.6, boss_ang=(45.0, 135.0, 225.0, 315.0))


def _elbow_dome():
    from . import arm
    from .belt_stage import bolted_stage_v4
    from .encoder import elbow_parts
    c = ELBOW_DOME
    zS = -arm.ELBOW_Z + arm.BELT_C
    st = {n: p for n, p, _ in bolted_stage_v4(arm.STAGE, 20, "e_", arm.MOT, arm.EAR_SITE)}
    can = st["e_can"] + st["e_can_cap"]
    import numpy as np
    v = np.array([(q.X, q.Y, q.Z) for q in can.vertices()])
    r_can = float(np.hypot(v[:, 0], v[:, 2] - zS).max())                            # ears (49.7)
    y_can = float(v[:, 1].min())                                                   # rear cap face (−65.4)
    ri, ye = r_can + c["clear"], y_can - c["end_gap"]
    y_top = -arm.HOUSING["y"] / 2 + c["skirt"]                                     # skirt end beside the housing's ±X faces
    ax = lambda y0, r, L: Pos(0, y0, zS) * Rot(-90, 0, 0) * Cylinder(r, L, align=MIN)
    d = ax(ye - WALL, ri + WALL, y_top - ye + WALL) - ax(ye, ri, y_top - ye + 1)
    for a in c["boss_ang"]:
        t = math.radians(a)
        x, z = c["boss_r"] * math.sin(t), zS - c["boss_r"] * math.cos(t)
        d += Pos(x, ye - 0.01, z) * Rot(-90, 0, 0) * Cylinder(c["boss_d"] / 2, -arm.HOUSING["y"] / 2 - 0.3 - ye, align=MIN)
        d -= Pos(x, ye - WALL - 1, z) * Rot(-90, 0, 0) * Cylinder(M3_CLEAR / 2, 40, align=MIN)
    d -= arm._housing_solid(-0.3)
    d -= arm.upper_bone()
    for n, p in [(n, p) for n, p, _ in elbow_parts()[0]] + [(n, p) for n, p, _ in elbow_parts()[1]]:
        b = p.bounding_box()
        d -= Pos((b.min.X + b.max.X) / 2, (b.min.Y + b.max.Y) / 2, (b.min.Z + b.max.Z) / 2) * Box(b.max.X - b.min.X + 2, b.max.Y - b.min.Y + 2, b.max.Z - b.min.Z + 2)
    d -= Pos(0, -arm.HOUSING["y"] / 2 - 30, -arm.ELBOW_Z) * Rot(-90, 0, 0) * Cylinder(22.0, 40, align=MIN)   # elbow shaft end (r 20) + 2
    return _main(d)


def arm_frame():
    """Rides on the upper bone.
    - Elbow bay dome over the elbow cartridge where it leaves the housing's −Y bay (_elbow_dome).
    - Collar door over the KX collar twist window: 2.2 mm, two snap lugs at its ends, 0.5 mm off the collar lugs.
    (An elbow side shield on the bone cap was tried: the forearm's inner face is not flat (the CF socket stands 7 mm
    proud of the link), so a static plane there makes a closing gap. R17 W18 (2026-11-28): a static shield can't close the
    elbow scissor at all (the housing is not round about the elbow axis beyond r ≈ 40, and the forearm sweeps 150° over
    it), so the forearm now stands arm.ELBOW_GAP (25.5 mm) off the housing's +Y face with a flat inner side, and a static
    round sleeve (arm.elbow_sleeve) covers the elbow shaft across the gap.)
    """
    from . import arm
    zS = -arm.ELBOW_Z + arm.BELT_C
    d = _elbow_dome()
    door = Pos(arm.HOUSING["x"] / 2 + 3.9, -20.5, zS) * soft_box(WALL + 0.6, 27, 98, 1.5, 1.2)          # 0.5 mm off the collar lugs (they stand 2 mm out of the window)
    for sy in (1, -1):                                                               # snap lugs at the door's ends, clear of the collar
        door += Pos(arm.HOUSING["x"] / 2 + 1.6, -20.5 + sy * 14.5, zS) * Box(3.2, 2.0, 20)
    return [("cover_elbow_bay", d, "white"), ("cover_collar_door", door, "white")]


# R17 (2026-11-28, rnd/pinch.py: arm_wrist_can ≈ 4,000 mm² exposed on both arms). The wrist boot (an 80 × 75 × 80 soft box
# with r 30 corners) stopped at the forearm's rim face (y 43) and was open toward the hand, so the wrist cartridge's
# Ø80 × 4 can flange (y 39…43, r 28.8…40, with the 6 screw heads) faced the hand in the open; at y 43 its rounded front
# was narrower (|x| ≈ 29) than the rim (r 40), so no ring could be joined to it there. The boot is now a surface of
# revolution about the wrist axis: a Ø88.4 cylinder (r 42 inside, 2 mm off the rim; the probe test reaches through
# walls nearer than ≈ 3.5 mm to a part) from a 2.2 mm front face
# (y 33…35.2, 0.8 mm in front of the screw heads, Ø54 bore round the palm boss) to a back wall 2 mm behind the wrist
# housing (y 98…100.2), cut round the forearm (the link web and the CF socket pass through it). The palm boss (r 24)
# turns 3 mm inside the bore (the output flange, r 25, is behind the face) and no other hand part comes nearer than
# 3 mm (they all lie below y 30 outside r 27), so every gap between the hand and the boot is the same at every wrist
# angle: wrist range and dexterity are unchanged. Two halves split at x = 0, 4 × M3 (bosses at r 38); they come off
# sideways with the hand on; for a cartridge swap the hand comes off first as before (assembly_seq removes covers first).
WRIST_BOOT = dict(bore=27.0, r_in=42.0, face=(33.0, 35.2), back=100.2, boss_r=38.0)


def _wrist_boot():
    from . import arm
    c = WRIST_BOOT
    ax = lambda y0, r, L: Pos(0, y0, -arm.FORE_L) * Rot(-90, 0, 0) * Cylinder(r, L, align=MIN)
    y0, y1 = c["face"]
    b = ax(y0, c["r_in"] + WALL, c["back"] - y0) - ax(y1, c["r_in"], c["back"] - WALL - y1)
    return b - ax(y0 - 1, c["bore"], y1 - y0 + 2)


def forearm_frame():
    """Rides on the forearm.
    - Forearm fairing: a C-section over the side link and its carbon tube, open toward the upper arm (the tube's inner
      face runs 3 mm past the housing face: a closed scissor gap, so no wall goes between). Two halves split at x = 0,
      M3 at both ends.
    - Elbow hub cap: Ø80 disc over the outer face of the forearm hub (closes the elbow-shaft end).
    - Wrist boot round the wrist cartridge, round about the wrist axis and closed toward the hand (_wrist_boot); two
      halves split at x = 0, 4 × M3.
    """
    from . import arm
    fo = arm.forearm()
    tube = arm.cf_fore()
    tx0, tx1, ty0, ty1, tz0, tz1 = _bb(tube)
    fy0, fy1 = ty0 - 1.0, max(ty1, arm.FORE_Y + 11) + 1.0 + WALL                       # inner edge 1 mm inboard of the tube face
    z0, z1 = tz0 - 6, tz1 + 6
    w = tx1 - tx0 + 2 + 2 * WALL
    fair = Pos(0, (fy0 + fy1) / 2, (z0 + z1) / 2) * soft_shell(w, fy1 - fy0, z1 - z0, 12, 4, WALL)
    fair -= Pos(0, fy0, (z0 + z1) / 2) * Box(w - 2 * WALL, 2 * WALL + 1, z1 - z0 - 2 * WALL)        # open toward the upper arm
    fair -= Pos(0, arm.FORE_Y, (z0 + z1) / 2) * Cylinder((tx1 - tx0) / 2 + 0.6, z1 - z0 + 2)         # the tube runs through
    fair -= fo
    fair = list(_halves(fair, "x", 0.0))
    for i, sx in enumerate((-1, 1)):                                                   # snap lugs at both ends
        for zz in (z0 + 4, z1 - 4):
            fair[i] = fair[i] + Pos(sx * 1.2, fy1 - WALL - 2, zz) * Box(2.0, 3.0, 5.0)
    hy = arm.FORE_Y + 11.0                                                             # forearm hub outer face (link 22 thick)
    hub_cap = Pos(0, hy + WALL / 2 + 0.2, 0) * Rot(-90, 0, 0) * (Cylinder(40, WALL) + Pos(0, 0, -WALL / 2 - 2) * (Cylinder(20.6, 4) - Cylinder(18.6, 4)))
    boot = _wrist_boot() - fo                                                          # R17: round, closed toward the hand
    c = WRIST_BOOT
    yb = (c["face"][0] + c["back"]) / 2
    boot = _split_cover(boot, "x", 0.0, [(yb - 22, -arm.FORE_L + c["boss_r"]), (yb + 22, -arm.FORE_L + c["boss_r"]),
                                         (yb - 22, -arm.FORE_L - c["boss_r"]), (yb + 22, -arm.FORE_L - c["boss_r"])])
    return [("cover_forearm_f", _main(fair[1]), "white"), ("cover_forearm_r", _main(fair[0]), "white"), ("cover_elbow_cap", hub_cap, "white"),
            ("cover_wrist_f", _main(boot[1]), "white"), ("cover_wrist_r", _main(boot[0]), "white")]


# ---------------------------------------------------------------------------------------------- lift (mast, belt)
# The concept had bellows above and below the torso. Over the 0.70 m stroke the low one would need ≈ 150 mm of
# collapsed stack (≈ 25 % of its length, ESTIMATE) that does not exist at the bottom of travel. Instead: the mast is a
# smooth static tube (no hazard by itself); the torso slides along it behind collars with a 5 mm gap; the only moving
# drive part outside the torso, the belt, runs in a static duct whose clamp slot is ≤ 7 mm (closed to a finger).
SLOT = 7.0                       # belt-clamp slot in the duct (≤ 8 mm, R17)


def _belt_xy():
    from . import upper
    return upper.lift_belt_xy()      # strand centre x, y; size along x, size along y (lift v2: belt in the YZ plane, 15 mm along x)


def belt_duct():
    """Static rectangular duct (2.2 mm PETG) round the lift belt strand from the lift cover to the mast cap, 4 mm round
    the belt; a SLOT-wide slot on its mast side lets the carriage's belt clamp through. Printed in 5 lengths
    (≤ 200 mm), spigot joints, M3 into the mast's rear face. Returned whole (one solid) for the checks."""
    from . import upper
    xb, yb, tb, wb = _belt_xy()
    z0, z1 = upper.LIFT_COVER_TOP, upper.Z_TOP - 30 + 4                       # stands on the lift cover's chimney (same footprint)
    ix, iy = tb + 8, wb + 8
    d = Pos(xb, yb, (z0 + z1) / 2) * (Box(ix + 2 * WALL, iy + 2 * WALL, z1 - z0) - Box(ix, iy, z1 - z0 + 2))
    d -= Pos(xb + ix / 2 + WALL / 2, yb, (z0 + z1) / 2) * Box(WALL + 2, SLOT, z1 - z0 + 2)
    return [("belt_duct", d, "white")]


def _duct_hole(z):
    from . import upper
    xb, yb, tb, wb = _belt_xy()
    return Pos(xb, yb, z) * Box(tb + 8 + 2 * WALL + 6, wb + 8 + 2 * WALL + 6, 40)     # duct + 3 mm a side


def _torso_box():
    from . import upper
    return _bb([p for n, p, _ in upper.torso(750.0) if n == "torso_shell"][0])


def torso_frame(lift_mm):
    """Rides on the lift (world frame at this lift height):
    - collars on the torso's top and bottom faces: close the mast passage (a 15 mm sliding gap round the mast) to
      5 mm round the mast and the ratchet strip, 3 mm round the belt duct. One plate each (≈ 120 × 110 mm), 4 × M3
      into the shell.
    - spine pod on the torso's back wall round the carriage's pawl box and belt clamp, from just under the pawl to the
      top collar; its floor closes 3 mm round the duct. One piece, 4 × M3 into the back wall.
    """
    from . import upper
    tx0, _, _, _, tz0, tz1 = _torso_box()
    xt = tx0                                                                            # torso back wall
    zb, zt = lift_mm + tz0 - 750.0, lift_mm + tz1 - 750.0
    pas = upper.W + 30
    xf = upper.MAST_X + pas / 2 + 10
    xb, yb, tb, wb = _belt_xy()
    xr = xb - (tb + 8) / 2 - WALL - 3 - WALL                                           # rear wall of the pod
    yw = max(pas + 20, wb + 8 + 2 * WALL + 2 * 3 + 2 * WALL + 40)
    out = []
    for n, z in (("collar_bot", zb - WALL), ("collar_top", zt)):
        xa = xr if n == "collar_top" else upper.MAST_X - pas / 2 - 10
        c = Pos((xa + xf) / 2, 0, z + WALL / 2) * soft_box(xf - xa, yw, WALL, 6, 0.6)
        c -= Pos(upper.MAST_X, 0, z) * Box(upper.W + 10, upper.W + 10, 20)
        c -= Pos(upper.MAST_X - upper.W / 2 - 5, upper.RACK["y"], z) * Box(20, upper.RACK["w"] + 10, 20)   # steel rack (A15) + 5 mm
        c -= _duct_hole(z)
        for sx, sy in ((xf - 8, yw / 2 - 8), (xf - 8, -yw / 2 + 8), (xt + 8, yw / 2 - 8), (xt + 8, -yw / 2 + 8)):
            c -= Pos(sx, sy, z) * Cylinder(M3_CLEAR / 2, 20)
        out.append((n, c, "white"))
    zp = lift_mm - 30 - 20 - 12                                                          # under the pawl (upper.carriage: pawl at lift − 30, 40 tall)
    pod = Pos((xr + xt + 2) / 2, 0, (zp + zt) / 2) * Box(xt + 2 - xr, yw, zt - zp)
    pod -= Pos((xr + WALL + xt + 2) / 2, 0, (zp + WALL + zt) / 2 + 1) * Box(xt + 2 - xr - WALL, yw - 2 * WALL, zt - zp - WALL + 2)
    pod -= _duct_hole(zp)
    car = upper.carriage(lift_mm)
    rack = Pos(upper.MAST_X - upper.W / 2 - 5, upper.RACK["y"], (zb + zt) / 2) * Box(20, upper.RACK["w"] + 10, zt - zb + 40)   # rack + 5 mm
    out.append(("spine_pod", _main(pod - car - rack), "white"))
    # the mast passage also cuts the torso's back wall: close that slot below the pod
    bp = Pos(xt - WALL / 2, 0, (zb + zp + WALL) / 2) * Box(WALL, pas + 4, zp + WALL - zb)
    out.append(("back_plate", _main(bp - car - rack), "white"))
    return out


def mast_cap():
    """Static, on the head bracket: sleeve round the bracket and the belt idler (it was open), top plate behind the
    bracket round the handle's feet, open below only round the mast and the belt duct. One piece, 2 × M3 into the
    bracket."""
    from . import upper
    xb, yb, tb, wb = _belt_xy()
    hd = {n: p for n, p, _ in upper.head()}
    x0 = min(xb - (tb + 8) / 2 - WALL - 6, _bb(hd["belt_idler"])[0] - 6)               # 6 mm behind the idler
    x1 = upper.MAST_X + upper.W / 2 + 6
    y0, y1 = -upper.W / 2 - 10, upper.W / 2 + 10
    z0, z1 = upper.Z_TOP - 30, upper.Z_TOP + 24
    c = Pos((x0 + x1) / 2, 0, (z0 + z1) / 2) * soft_shell(x1 - x0 + 2 * WALL, y1 - y0 + 2 * WALL, z1 - z0, 8, 1, WALL)
    c -= Pos(upper.MAST_X, 0, z0) * Box(upper.W + 2, upper.W + 2, 2 * WALL + 1)        # mast (1 mm, static)
    # (lift v2: the steel rack ends at z 1095, below the cap, so its floor is closed there)
    c -= Pos(xb, yb, z0) * Box(tb + 8 + 2 * WALL + 1, wb + 8 + 2 * WALL + 1, 2 * WALL + 1)   # duct
    c -= Pos(upper.MAST_X, 0, z1) * Box(upper.W + 16.8, upper.W + 16.8, 2 * WALL + 1)  # bracket top stays bare (pan sleeve)
    hd = {n: p for n, p, _ in upper.head()}
    c -= hd["handle"]
    c -= hd["head_bracket"]
    return [("mast_cap", c, "white")]


def lift_covers(lift_mm):
    return torso_frame(lift_mm) + mast_cap() + belt_duct()


# ---------------------------------------------------------------------------------------------- base
def base_frame():
    """Static: belly pan under the deck (closes the wheel cartridges to a finger through the 25 mm floor gap) and a
    liner round each wheel well (wheel + 6 mm), its inner wall with a hole for the wheel shaft (+3 mm). Sized from
    the wheel and pod CAD. The pan prints as 4 quarters (each ≤ 256 mm), M3 into the deck's pod bosses."""
    from . import base
    parts = {n: p for n, p, _ in base.module()}
    zf = base.D.base.clearance * 1000
    pan = Cylinder(base.R_SHELL - 4, WALL, align=MIN)
    out = []
    wells, keep = [], []
    for n, p in parts.items():
        if n.endswith("_wheel"):
            x0, x1, y0, y1, z0, z1 = _bb(p)
            zd = min(_bb(q)[4] for m, q in parts.items() if m.startswith("deck"))          # deck underside
            well = Pos((x0 + x1) / 2, (y0 + y1) / 2, zf + (zd - zf) / 2) * Box(x1 - x0 + 12, y1 - y0 + 12, zd - zf + 0.02)
            liner = Pos((x0 + x1) / 2, (y0 + y1) / 2, zf + (zd - zf) / 2) * Box(x1 - x0 + 12 + 2 * WALL, y1 - y0 + 12 + 2 * WALL, zd - zf) - well
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2                                         # side walls run on out to the skirt
            rr = math.hypot(cx, cy)
            ux, uy = cx / rr, cy / rr
            ext = base.R_SHELL - (rr + (x1 - x0 if abs(ux) > 0.5 else y1 - y0) / 2 + 6)
            for sgn in (1, -1):
                o = (y1 - y0 if abs(ux) > 0.5 else x1 - x0) / 2 + 6 + WALL / 2
                px, py = cx + ux * ((x1 - x0 if abs(ux) > 0.5 else y1 - y0) / 2 + 6 + ext / 2) - uy * sgn * o, cy + uy * ((x1 - x0 if abs(ux) > 0.5 else y1 - y0) / 2 + 6 + ext / 2) + ux * sgn * o
                sx_, sy_ = (ext, WALL) if abs(ux) > 0.5 else (WALL, ext)
                liner += Pos(px, py, zf + (zd - zf) / 2) * Box(sx_, sy_, zd - zf)
            sh = parts[n.replace("_wheel", "_shaft")]
            sx0, sx1, sy0, sy1, sz0, sz1 = _bb(sh)
            ax = "x" if abs(ux) > 0.5 else "y"                                          # the shaft is radial (its flange makes the box wider than long)
            r = (sz1 - sz0) / 2 + 3
            c = ((sx0 + sx1) / 2, (sy0 + sy1) / 2, (sz0 + sz1) / 2)
            liner -= Pos(*c) * (Rot(0, 90, 0) if ax == "x" else Rot(90, 0, 0)) * Cylinder(r, (sx1 - sx0 if ax == "x" else sy1 - sy0) + 40)   # R17: through the inner wall only (was 400 mm along the tangent: a Ø66 hole through both side walls and the inner wall, the probe's way into the base)
            ang = math.degrees(math.atan2((y0 + y1) / 2, (x0 + x1) / 2))
            wy = (y1 - y0) if ax == "x" else (x1 - x0)                                  # wheel diameter (tangential)
            liner -= Rot(0, 0, ang) * Pos(base.R_SHELL, 0, zf + (z1 - zf) / 2) * Box(40, wy + 12, z1 - zf)   # window where the wheel leaves the skirt
            for m, q in parts.items():
                if m.startswith(("skirt", "bumper", n[:-6] + "_pod")):
                    liner -= q
            wells.append(well)
            out.append((f"liner_{n[:-6]}", _main(liner), "white"))
        if n.endswith("_pod"):
            keep.append(p)
    pan = Pos(0, 0, zf) * pan
    for w in wells:
        pan -= w
    for p in keep:
        pan -= p                                                                       # round the pod solid (its frame is open below)
    for w in getattr(base, "caster_wells", lambda: [])():                             # A10 v2 D18b casters: the cup passes the pan, 2 mm a side
        pan -= w
    for q in range(4):
        quarter = pan & (Rot(0, 0, 90 * q + 45) * Pos(base.R_SHELL / 2 + 0.15, base.R_SHELL / 2 + 0.15, zf) * Box(base.R_SHELL, base.R_SHELL, 20))
        out.append((f"belly_{q}", _main(quarter), "white"))
    return out


# ---------------------------------------------------------------------------------------------- compliant (TPU) covers
def _roll_inboard_spans():
    """Design study, shoulder roll (R17 wave 5): a TPU 95A fan bellows across the inboard wedge between the static
    roll body (ferrule flange Ø91 / roll-link frame, lowest edge at φ_s) and the yaw block's inboard top edge (φ_b + roll),
    pleats radiating from the roll axis (φ: 0 = −Z, + toward +Y). Developed length at the block's edge radius; floor 2 mm
    (the closest approach, rnd/parts/roll.ferrule_sweep_gap ≥ 2). The wedge goes from ≈ 2 mm at −15° to ≈ 135 mm at
    100°, so no pleat pitch keeps it within ±40 % (a printed fan would stack solid at −15° and be the pinch itself)."""
    from . import roll
    from .shoulder import FERRULE
    r = math.hypot(46.0, -roll.P1[1])                                               # block inboard top edge (plates ±46, top at P1)
    phi_b = -math.degrees(math.atan2(46.0, -roll.P1[1]))                            # −41.5°
    phi_s = math.degrees(math.atan2(-(roll.ROLL_Y - 1.5), FERRULE["od"] / 2))        # ferrule flange's lowest edge, −55.4°
    f = lambda a: max(2.0, r * math.radians(a + phi_b - phi_s))
    lo, hi = roll.ROLL_MIN, 100.0
    L0 = (f(lo) + f(hi)) / 2                                                        # best case: as printed at mid-stroke
    return [("fan, block edge → ferrule (arc at r %.0f)" % r, L0, f)]


def compliant():
    """Compliant (TPU 95A) covers for rnd/pinch.py's rule: [{name, part (robot part-name suffix; None = design study),
    joint, wall (mm), spans: [(name, free length mm, length(joint deg) mm)], note}]."""
    return [dict(name="shoulder roll inboard fan bellows", part=None, joint="roll", wall=1.6, spans=_roll_inboard_spans(),
                 note="not buildable: needs the wedge ≥ 25 mm at −15° (or ROLL_Y ≈ +23 mm); closed instead by the D25 people-near "
                      "roll soft limit (twin.JR_PEOPLE_NEAR, sw/safety.py): ≥ 25 mm from ≈ +1°, limit +5°; "
                      "the outer sector is closed by the yaw hood (rigid)")]


# ---------------------------------------------------------------------------------------------- printable list
def printed():
    """{piece: solid} for the printability check (rigid pieces in PETG, gaiters in TPU 95A)."""
    from . import upper
    P = {}
    for f in (arm_frame, forearm_frame, mast_cap, base_frame):
        for n, p, _ in f():
            P[f"cover {n.removeprefix('cover_')}"] = p
    for n, p, _ in torso_frame(750.0):
        P[f"cover {n}"] = p
    from . import roll
    if roll.INLINE:
        P["cover yaw hood (inline roll)"] = _yaw_hood()
        P["cover roll front collar (inline roll)"] = roll_front_collar()
        from ..params import WEDGE_GUARD
        if WEDGE_GUARD == "skirt":
            P["cover yaw hood TPU skirt (PIP_WEDGE_GUARD=skirt, TPU 95A)"] = wedge_skirt()
    duct = belt_duct()[0][1]
    x0, x1, y0, y1, z0, z1 = _bb(duct)
    k = math.ceil((z1 - z0) / 200)
    for i in range(k):
        za, zb = z0 + (z1 - z0) * i / k, z0 + (z1 - z0) * (i + 1) / k
        P[f"cover belt_duct [{i + 1}/{k}]"] = duct & (Pos((x0 + x1) / 2, (y0 + y1) / 2, (za + zb) / 2) * Box(x1 - x0 + 2, y1 - y0 + 2, zb - za - 0.2))
    return P
