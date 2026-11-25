"""Soft forms for the industrial-design pass: boxes and shells with every edge rounded (pill-like, robot-friendly)."""
from __future__ import annotations

from build123d import Axis, Box, fillet


def soft_box(x, y, z, rv, re=None):
    """Box with vertical edges rounded by rv and top/bottom edges by re (≤ rv)."""
    rv = min(rv, x / 2 - 0.4, y / 2 - 0.4)
    re = min(re if re is not None else rv * 0.6, z / 2 - 0.4, rv - 0.2)
    b = Box(x, y, z)
    if rv > 0.5:
        b = fillet(b.edges().filter_by(Axis.Z), rv)
    if re > 0.5:
        try:
            b = fillet(b.edges().group_by(Axis.Z)[0] + b.edges().group_by(Axis.Z)[-1], re)
        except ValueError:
            pass
    return b


def soft_shell(x, y, z, rv, re=None, t=2.0):
    re = re if re is not None else rv * 0.6
    return soft_box(x, y, z, rv, re) - soft_box(x - 2 * t, y - 2 * t, z - 2 * t, max(rv - t, 1), max(re - t, 0.6))
