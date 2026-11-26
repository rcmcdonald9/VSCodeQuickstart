"""Torque-reaction pads for small cartridges (loop 25): find a can's torque pin from its seat geometry and build a
slotted pad around the pin's tip, fused to the part the cartridge is mounted on.

The pad is a block (radial × tangential × axial) centred on the pin's protruding length. A slot (pin Ø + 0.4) runs
radially through it, open toward the cartridge axis so the cartridge keeps its radial float. If the pad doesn't touch the
anchor part, a Ø10 bridge joins it to the anchor's nearest point.
"""
from __future__ import annotations

import numpy as np
from build123d import Box, Cylinder, Part, Plane, Pos


def _seat(part, r_seat, tol=0.06):
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Cylinder
    for f in part.faces():
        s = BRepAdaptor_Surface(f.wrapped)
        if s.GetType() == GeomAbs_Cylinder and abs(s.Cylinder().Radius() - r_seat) < tol:
            ax = s.Cylinder().Axis()
            d = np.array([ax.Direction().X(), ax.Direction().Y(), ax.Direction().Z()])
            loc = np.array([ax.Location().X(), ax.Location().Y(), ax.Location().Z()])
            c = np.array([f.center().X, f.center().Y, f.center().Z])
            return loc + np.dot(c - loc, d) * d, d
    raise ValueError("no torque-pin seat found")


def pin_frame(can, r_seat):
    """(seat centre on the pin axis, outward pin direction, radial unit vector from the cartridge axis)."""
    c, ax = _seat(can, r_seat)
    cen = np.array([can.center().X, can.center().Y, can.center().Z])
    out = ax if np.dot(c - cen, ax) > 0 else -ax
    rad = (c - cen) - np.dot(c - cen, out) * out
    return c, out, rad / np.linalg.norm(rad)


def with_pin_slot(can, anchor, r_seat, r_pin, floor=2.5, z0=3.0, t=6.0, width=14.0, out_len=6.0, in_len=10.0) -> Part:
    """`anchor` + a slotted pad around the pin (z0 … z0 + t beyond the seat centre along the pin) + a Ø10 bridge if the pad
    doesn't touch the anchor; the slot is cut through pad *and* anchor (from the can's rear face outward), open toward the
    cartridge axis, and the can itself is kept clear."""
    c, out, rad = pin_frame(can, r_seat)
    pl = Plane(origin=tuple(c + out * z0), x_dir=tuple(rad), z_dir=tuple(out))
    pad = pl.location * (Pos((out_len - in_len) / 2, 0, 0) * Box(out_len + in_len, width, t, align=(_c(), _c(), _min())))
    d, p_pad, p_anc = pad.distance_to_with_closest_points(anchor)
    if d > 0.05:
        a, b = np.array([p_pad.X, p_pad.Y, p_pad.Z]), np.array([p_anc.X, p_anc.Y, p_anc.Z])
        v = (b - a) / np.linalg.norm(b - a)
        pad = pad + Plane(origin=tuple(a - v * 1.5), z_dir=tuple(v)).location * Cylinder(5, d + 3.0, align=(_c(), _c(), _min()))
    s0 = floor / 2 + 0.2                                             # from just behind the can's rear face
    slot = Plane(origin=tuple(c + out * s0), x_dir=tuple(rad), z_dir=tuple(out)).location * (
        Pos((r_pin + 1.5 - in_len - 2) / 2, 0, 0) * Box(r_pin + 1.5 + in_len + 2, 2 * r_pin + 0.4, z0 + t + 0.5 - s0, align=(_c(), _c(), _min())))
    return anchor + pad - slot - can


def _c():
    from build123d import Align
    return Align.CENTER


def _min():
    from build123d import Align
    return Align.MIN
