"""KX joint interface v1: a kinematic seat plus a spring-preloaded breech-lock collar.

Precedents: Maxwell 3-groove kinematic couplings built from a ball on two rods (precision
fixturing); breech-lock and PL lens mounts (twist collar, sprung preload); shaft-mounted
gear reducers held by a torque arm (SEW, e-bike hub motors).

**Load path (v1).** A cartridge carries **torque only**.
- Its output face (male) locks into a **hub** in the driven bone. The hub turns in the
  bone's own bearings, which carry every bending and axial load.
- The motor housing is kept from spinning by a **torque pin** sitting in a radial slot.
  The slot leaves radial and axial freedom, so the cartridge is never over-constrained.

**Parts.**
    male   (on the cartridge output): flange with 3 hardened balls (pressed in) and 3 bayonet lugs
    hub    (in the bone): 3 grooves, each made of two steel dowel pins laid radially
           (a ball on two rods). This is not a printed V-groove: the Hertz contact stress
           on plastic would be about 5x PETG yield (see docs/rnd/kx.md).
    collar (captive on the hub): internal ramped teeth. A 35° twist puts the teeth behind
           the lugs. A spring stack between the collar lip and the hub step sets the preload.
    retainer: screwed under the hub; it keeps the collar and spring captive.

Coordinates in **mated** position: the hub's top face is z = 0; the male arrives from +Z with
its mating face down; the ball centres sit at z = z_ball. All lengths are mm.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from build123d import Align, Box, Cylinder, Part, Pos, Rot, Sphere, Axis, chamfer

MIN = (Align.CENTER, Align.CENTER, Align.MIN)


@dataclass(frozen=True)
class KX:
    name: str
    od: float           # male flange OD
    face_t: float       # male flange thickness
    ball_d: float       # hardened steel ball (G25 chrome steel)
    ball_pcd: float
    pin_d: float        # hardened dowel pins forming each groove
    pin_len: float
    lug_reach: float    # lug radial extension beyond od/2
    lug_w: float        # lug tangential width
    lug_t: float        # lug axial thickness (at the back of the flange)
    tooth_t: float      # collar tooth axial thickness
    preload_n: float    # design preload from the spring stack
    beta_deg: float = 20.0      # ball-to-pin contact normal from the face plane. Lower = less lift-out per N·m
                                # -> less preload -> easier twist (sweep in docs/rnd/kx.md: 30° needed 250 N, 20° needs 160 N)
    pin_proud: float = 0.6      # pins stand this far above the hub face (sets the face gap); male face has reliefs
    twist_deg: float = 35.0
    ramp_deg: float = 6.0       # below the friction angle (8.5° lubed, 14° dry PETG) -> self-locking
    clearance: float = 0.3
    spring_t: float = 1.2       # spring stack (wave spring or 6 small coil springs) installed height
    stroke: float = 1.0         # teeth sit this far below the lug backs: the last ~14° of twist lifts
                                # the collar by `stroke`, compressing the spring -> preload (k ≈ preload / 1.5 mm)

    # ---- derived geometry ----
    @property
    def r_ball(self): return self.ball_pcd / 2

    @property
    def pin_z(self): return -self.pin_d / 2 + self.pin_proud

    @property
    def z_ball(self):
        return self.pin_z + (self.ball_d + self.pin_d) / 2 * math.sin(math.radians(self.beta_deg))

    @property
    def pin_sep(self):                                         # pin centre spacing
        return (self.ball_d + self.pin_d) * math.cos(math.radians(self.beta_deg))

    @property
    def gap(self): return self.z_ball                           # male face height above hub face

    @property
    def lug_back_z(self): return self.gap + self.face_t        # back of the male flange / lugs (mated)

    @property
    def r_collar_in(self): return self.od / 2 + self.lug_reach + self.clearance + 0.4

    @property
    def r_hub(self): return self.r_collar_in - 0.3

    @property
    def r_collar_out(self): return self.r_collar_in + 5.5

    @property
    def r_tooth_in(self): return self.od / 2 + self.clearance + 0.2   # teeth must clear the flange while inserting


# Cartridge M (GIM6010-8, DM-J4310). Cartridge S (STS3215).
KX74 = KX("KX74", od=74, face_t=7, ball_d=10.0, ball_pcd=58, pin_d=5.0, pin_len=14,
          lug_reach=6, lug_w=14, lug_t=3.0, tooth_t=2.4, preload_n=160)
# DM-J4310 (Ø57, 7 N·m peak): KX74 is oversized for the wrist and upper-arm yaw (loop 4 mass pass)
KX56 = KX("KX56", od=56, face_t=6, ball_d=8.0, ball_pcd=43, pin_d=4.0, pin_len=11,
          lug_reach=5, lug_w=11, lug_t=2.5, tooth_t=2.2, preload_n=140)
KX44 = KX("KX44", od=44, face_t=5, ball_d=6.0, ball_pcd=31, pin_d=3.0, pin_len=8,
          lug_reach=4, lug_w=9, lug_t=2.5, tooth_t=2.0, preload_n=80)
BALL_ANG = (90, 210, 330)
LUG_ANG = (30, 150, 270)
HUB_STEP = 6.0       # hub keeps r_hub from z=0 to -HUB_STEP, then steps in by STEP_IN
STEP_IN = 1.5
LIP_T = 2.0


def _at(r, ang):
    return Pos(r * math.cos(math.radians(ang)), r * math.sin(math.radians(ang)), 0)


def male(kx: KX, bore: float = 0.0) -> Part:
    """Male face in its own frame: z in [0, face_t], mating face at z = 0 (normal -Z), lugs at the back."""
    p = Cylinder(kx.od / 2, kx.face_t, align=MIN)
    p = chamfer(p.edges().group_by(Axis.Z)[0], 0.6)       # lead-in on the mating edge
    for a in LUG_ANG:
        # back face ramped like the collar teeth -> face contact, not an edge
        p += _at(kx.od / 2 + kx.lug_reach / 2 - 1, a) * Rot(0, 0, a) * Pos(0, 0, kx.face_t) * Rot(-kx.ramp_deg, 0, 0) * \
            Box(kx.lug_reach + 2, kx.lug_w, kx.lug_t, align=(Align.CENTER, Align.CENTER, Align.MAX))
    for a in BALL_ANG:                                    # press-fit pockets, ball centre on the face
        p -= _at(kx.r_ball, a) * Sphere(kx.ball_d / 2 - 0.05)
        # relief so the proud pins clear the face
        p -= _at(kx.r_ball, a) * Rot(0, 0, a) * Box(kx.pin_len + 2, kx.pin_sep + kx.pin_d + 2, 2 * (kx.pin_proud + 0.5))
    if bore:
        p -= Cylinder(bore / 2, 100)
    return p


def balls(kx: KX) -> Part:
    out = None
    for a in BALL_ANG:
        b = _at(kx.r_ball, a) * Sphere(kx.ball_d / 2)
        out = b if out is None else out + b
    return out


def _pin(kx: KX, a: float, side: int) -> Part:
    """One dowel pin, radial, offset tangentially by ±pin_sep/2 (mated frame)."""
    return Rot(0, 0, a) * Pos(kx.r_ball, side * kx.pin_sep / 2, kx.pin_z) * Rot(0, 90, 0) * \
        Cylinder(kx.pin_d / 2, kx.pin_len)


def pins(kx: KX) -> Part:
    out = None
    for a in BALL_ANG:
        for s in (-1, 1):
            out = _pin(kx, a, s) if out is None else out + _pin(kx, a, s)
    return out


def hub(kx: KX, depth: float = None, bore: float = 0.0) -> Part:
    """Female hub (top of a bone's driven member). Face at z = 0, body down to -depth."""
    depth = depth or HUB_STEP + LIP_T + kx.spring_t + 1.0
    p = Pos(0, 0, -HUB_STEP) * Cylinder(kx.r_hub, HUB_STEP, align=MIN)
    p += Pos(0, 0, -depth) * Cylinder(kx.r_hub - STEP_IN, depth - HUB_STEP, align=MIN)
    for a in BALL_ANG:
        # pin bores (pressed in radially from the outside; the access hole is the same size)
        for s in (-1, 1):
            p -= Rot(0, 0, a) * Pos((kx.r_ball - kx.pin_len / 2 + kx.r_hub + 2) / 2, s * kx.pin_sep / 2, kx.pin_z) * \
                Rot(0, 90, 0) * Cylinder(kx.pin_d / 2, kx.r_hub + 2 - (kx.r_ball - kx.pin_len / 2))
        # relief channel: exposes the pin contact lines; deeper slot clears the ball's bottom
        p -= Rot(0, 0, a) * Pos(kx.r_ball, 0, 0) * Box(kx.pin_len + 1, kx.pin_sep, -kx.pin_z * 2)
        p -= Rot(0, 0, a) * Pos(kx.r_ball, 0, 0) * Box(kx.ball_d, kx.pin_sep - kx.pin_d, (kx.ball_d / 2 - kx.z_ball + 0.8) * 2)
    # 3 × M3 holes for the retainer
    for a in LUG_ANG:
        p -= _at(kx.r_hub - STEP_IN - 3.5, a) * Pos(0, 0, -depth) * Cylinder(1.25, 8, align=MIN)
    if bore:
        p -= Cylinder(bore / 2, 100)
    return p


def retainer(kx: KX, depth: float = None) -> Part:
    depth = depth or HUB_STEP + LIP_T + kx.spring_t + 1.0
    r = Pos(0, 0, -depth - 2) * (Cylinder(kx.r_collar_in + 1.5, 2, align=MIN) - Cylinder(kx.r_hub - STEP_IN - 7, 2, align=MIN))
    for a in LUG_ANG:
        r -= _at(kx.r_hub - STEP_IN - 3.5, a) * Pos(0, 0, -depth - 2) * Cylinder(1.7, 2, align=MIN)
    return r


def spring(kx: KX) -> Part:
    """Placeholder for the preload spring stack (sits under the hub step, above the collar lip)."""
    return Pos(0, 0, -HUB_STEP - kx.spring_t) * (Cylinder(kx.r_hub - 0.2, kx.spring_t, align=MIN) -
                                                 Cylinder(kx.r_hub - STEP_IN + 0.3, kx.spring_t, align=MIN))


def collar(kx: KX, locked: bool = True) -> Part:
    """Collar in the mated frame. Unlocked = rotated +twist_deg (the teeth sit between the lugs)."""
    z0 = -HUB_STEP - kx.spring_t - LIP_T
    top = kx.lug_back_z + kx.tooth_t
    c = Pos(0, 0, z0) * (Cylinder(kx.r_collar_out, top - z0, align=MIN) - Cylinder(kx.r_collar_in, top - z0, align=MIN))
    c += Pos(0, 0, z0) * (Cylinder(kx.r_collar_in + 0.1, LIP_T, align=MIN) - Cylinder(kx.r_hub - STEP_IN + 0.3, LIP_T, align=MIN))
    reach = kx.r_collar_in - kx.r_tooth_in
    for a in LUG_ANG:
        # tooth underside slopes by ramp_deg: its leading edge (toward the lug) is higher
        t = Pos(0, 0, kx.lug_back_z - kx.stroke) * Rot(-kx.ramp_deg, 0, 0) * Box(reach + 1.0, kx.lug_w - 2, kx.tooth_t, align=MIN)
        c += Rot(0, 0, a) * Pos(kx.r_tooth_in + (reach + 1.0) / 2, 0, 0) * t
    for k in range(24):                                   # grip flutes
        c -= _at(kx.r_collar_out + 0.5, k * 15) * Pos(0, 0, z0) * Cylinder(1.1, top - z0, align=MIN)
    for a in (LUG_ANG[0] + 60, LUG_ANG[1] + 60):          # two thumb tabs for leverage
        c += Rot(0, 0, a) * Pos(kx.r_collar_out + 3, 0, z0) * Box(9, 10, top - z0 - 2, align=MIN)
    return c if locked else Rot(0, 0, kx.twist_deg) * c


def male_mated(kx: KX, bore: float = 0.0) -> Part:
    return Pos(0, 0, kx.gap) * male(kx, bore)


def balls_mated(kx: KX) -> Part:
    return Pos(0, 0, kx.gap) * balls(kx)
