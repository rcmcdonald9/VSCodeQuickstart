"""Printed HTD 5M pulleys and the small-pulley hub (KX74 socket + straddle-mounted pulley on 2 bearings).

Load path: the cartridge (torque only) locks into the hub's KX74 face; belt pull goes into two 61804 bearings
(20×32×7) that sit in the bone walls on both sides of the pulley. Tooth form is an approximation of HTD 5M
(round groove, depth 2.06 mm, pitch-line offset 0.57 mm): print a test pulley and check it against a real belt.
"""
from __future__ import annotations

import math

from build123d import Align, Box, Cylinder, Part, Pos, Rot

from .kx import KX74, hub

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


def belt_loop(pd_small: float, pd_big: float, centre: float, width: float = 15.0, t: float = 3.6) -> Part:
    """Belt as a solid band around two pitch circles (for renders): hull of two discs minus the inner hull."""
    def hull(r1, r2):
        a = math.asin((r2 - r1) / centre)
        L = math.hypot(centre, 0)
        body = Cylinder(r1, width, align=MIN) + Pos(centre, 0, 0) * Cylinder(r2, width, align=MIN)
        # quadrilateral between tangent points
        mid = (r1 + r2) / 2
        body += Pos(centre / 2, 0, 0) * Box(centre, 2 * mid * math.cos(a) * 0.999, width, align=MIN)
        return body
    return hull(pd_small / 2 + t / 2, pd_big / 2 + t / 2) - hull(pd_small / 2 - t / 2, pd_big / 2 - t / 2)
