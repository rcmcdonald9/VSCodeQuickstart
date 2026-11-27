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
    (≈ 6 cm³ of overlap); the inline roll's static cover belongs with the shoulder boot (open, see roll_frame)."""
    from . import roll
    if roll.INLINE:
        return []
    x0, x1, y0, y1, z0, z1 = -122.0, 62.0, 4.0, roll.ROLL_Y + 72, -30.0, roll.ROLL_C + 78
    d = Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * soft_shell(x1 - x0, y1 - y0, z1 - z0, 46, 34, 2.2)
    d -= Pos((x0 + x1) / 2, (y0 + y1) / 2, z0) * Box(x1 - x0 - 6, y1 - y0 - 6, 8)
    d -= Pos(0, roll.ROLL_Y, -30) * Box(x1 - x0 + 10, 120, 70)                         # the arm exits below the roll axis
    return [("cover_shoulder", d, "white")]


_HOOD = []
_GROW = [(0, 0, 0)] + [tuple(s * 1.5 * (i == k) for i in range(3)) for k in range(3) for s in (1, -1)]   # ≈ 1.5 mm growth


def _yaw_hood():
    """D24 inline roll (2026-11-27): rides on the roll output. A 2.2 mm hood 3 mm round the bolted yaw cartridge (can +
    rear cap), standing on the yaw block's top plate (the yaw belt is already inside the yaw band). Cut clear of the
    static roll link, motor housing, ferrule and spring ring swept over the roll range (3° steps, ±1.5 mm on each axis), and 1.5 mm
    off the roll output's rear arm; it stops 1.5 mm behind the roll link (x −46.5), leaving the can's front 10 mm open. Concept (render + pinch), not yet a print: the cut leaves it open toward the roll
    link, which is where the TPU shoulder boot has to close the joint."""
    if not _HOOD:
        from . import roll
        from .. import swept
        mv, st, o, _, (lo, hi) = swept.joints()["roll"]
        can = [p for n, p, _ in roll.module() if n == "yaw_can"][0].bounding_box()
        t, g = WALL, 3.0
        x0, x1, y0, y1 = can.min.X - g - t, can.max.X + g + t, can.min.Y - g - t, can.max.Y + g + t
        z0, z1 = roll.ZF + 0.5, can.max.Z + g + t
        h = Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * soft_shell(x1 - x0, y1 - y0, z1 - z0, 14, 8, t)
        h -= Pos((x0 + x1) / 2, (y0 + y1) / 2, z0) * Box(x1 - x0 - 2 * t, y1 - y0 - 2 * t, 2 * t + 1)
        R = lambda a: Pos(0, o[1], o[2]) * Rot(a, 0, 0) * Pos(0, -o[1], -o[2])
        hx = h.bounding_box().max.X
        for a in range(int(lo) - 3, int(hi) + 4, 3):
            for _, p in st:
                for d in _GROW:
                    q = Pos(*d) * (R(-a) * p)
                    if q.bounding_box().min.X <= hx:
                        h -= q
        for n, p in mv:
            if n == "roll_arm_rear":
                for d in _GROW:
                    h -= Pos(*d) * p
        h -= Pos(-46.5 + 100, 0, 0) * Box(200, 600, 600)                       # stop 1.5 mm behind the roll link / rear arm (x −45)
        _HOOD.append(_main(h))
    return _HOOD[0]


def roll_frame():
    """Concept: rides on the roll output: a sleeve round the whole yaw block (both plates, the belt, the yaw
    cartridge, its KX collar and torque fork), open at the top (the clevis arms) and the bottom (the yaw flange).
    Inline roll (default): the yaw-cartridge hood (_yaw_hood); the old sleeve's top followed ZF + ZC and it engulfed the
    roll link and motor (≈ 30 cm³ of overlap)."""
    from . import roll
    if roll.INLINE:
        return [("cover_yaw", _yaw_hood(), "white")]
    z0, z1 = roll.P2[0] - 4, roll.ZF + roll.ZC + 14
    c = Pos(-38, roll.ROLL_Y, (z0 + z1) / 2) * soft_shell(198, 122, z1 - z0, 46, 18, 2.2)
    c -= Pos(-38, roll.ROLL_Y, z0) * Box(194, 118, 6)
    c -= Pos(0, roll.ROLL_Y, z1) * Box(80, 70, 8)                                       # clevis arms rise through the top
    return [("cover_yaw", c, "white")]


# ---------------------------------------------------------------------------------------------- elbow + forearm
def arm_frame():
    """Rides on the upper bone.
    - Elbow bay dome over the elbow cartridge where it leaves the housing's −Y bay (the concept shell, 2.2 mm): 4 M3
      bosses inside its corners, screws through the dome's back into inserts in the housing wall; one piece, goes on
      from −Y.
    - Collar door over the KX collar twist window: 2.2 mm, two snap lugs at its ends, 0.5 mm off the collar lugs.
    (An elbow side shield on the bone cap was tried: the forearm's inner face is not flat (the CF socket stands 7 mm
    proud of the link), so a static plane there makes a closing gap. The elbow scissor needs a flat forearm inner
    face in arm.py.)
    """
    from . import arm
    zS = -arm.ELBOW_Z + arm.BELT_C
    d = Pos(0, -70, zS) * soft_shell(104, 40, 104, 30, 14, WALL)
    d -= Pos(0, -52, zS) * Box(108, 6, 108)                                         # open against the housing wall
    for sx in (1, -1):
        for sz in (1, -1):
            d += Pos(sx * 42, -70, zS + sz * 42) * Rot(90, 0, 0) * (Cylinder(4.5, 36) - Cylinder(M3_CLEAR / 2, 36))
    door = Pos(arm.HOUSING["x"] / 2 + 3.9, -20.5, zS) * soft_box(WALL + 0.6, 27, 98, 1.5, 1.2)          # 0.5 mm off the collar lugs (they stand 2 mm out of the window)
    for sy in (1, -1):                                                               # snap lugs at the door's ends, clear of the collar
        door += Pos(arm.HOUSING["x"] / 2 + 1.6, -20.5 + sy * 14.5, zS) * Box(3.2, 2.0, 20)
    return [("cover_elbow_bay", d, "white"), ("cover_collar_door", door, "white")]


def forearm_frame():
    """Rides on the forearm.
    - Forearm fairing: a C-section over the side link and its carbon tube, open toward the upper arm (the tube's inner
      face runs 3 mm past the housing face: a closed scissor gap, so no wall goes between). Two halves split at x = 0,
      M3 at both ends.
    - Elbow hub cap: Ø80 disc over the outer face of the forearm hub (closes the elbow-shaft end).
    - Wrist boot round the wrist cartridge, open toward the hand; two halves split at x = 0, 4 × M3.
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
    boot = Pos(0, (arm.WRIST_Y0 + arm.WRIST_Y1) / 2 + 4, -arm.FORE_L) * soft_shell(80, arm.WRIST_Y1 - arm.WRIST_Y0 + 22, 80, 30, 18, WALL)
    boot -= Pos(0, arm.WRIST_Y0 - 8, -arm.FORE_L) * Box(90, 16, 90)                     # open toward the hand
    boot -= fo
    yb = (arm.WRIST_Y0 + arm.WRIST_Y1) / 2 + 4
    boot = _split_cover(boot, "x", 0.0, [(yb - 20, -arm.FORE_L + 32), (yb + 20, -arm.FORE_L + 32),
                                         (yb - 20, -arm.FORE_L - 32), (yb + 20, -arm.FORE_L - 32)])
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
    out.append(("spine_pod", _main(pod - car), "white"))
    # the mast passage also cuts the torso's back wall: close that slot below the pod
    bp = Pos(xt - WALL / 2, 0, (zb + zp + WALL) / 2) * Box(WALL, pas + 4, zp + WALL - zb)
    out.append(("back_plate", _main(bp - car), "white"))
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
    for q in range(4):
        quarter = pan & (Rot(0, 0, 90 * q + 45) * Pos(base.R_SHELL / 2 + 0.15, base.R_SHELL / 2 + 0.15, zf) * Box(base.R_SHELL, base.R_SHELL, 20))
        out.append((f"belly_{q}", _main(quarter), "white"))
    return out


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
    duct = belt_duct()[0][1]
    x0, x1, y0, y1, z0, z1 = _bb(duct)
    k = math.ceil((z1 - z0) / 200)
    for i in range(k):
        za, zb = z0 + (z1 - z0) * i / k, z0 + (z1 - z0) * (i + 1) / k
        P[f"cover belt_duct [{i + 1}/{k}]"] = duct & (Pos((x0 + x1) / 2, (y0 + y1) / 2, (za + zb) / 2) * Box(x1 - x0 + 2, y1 - y0 + 2, zb - za - 0.2))
    return P
