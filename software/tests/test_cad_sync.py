"""cad/params.scad and config.yaml describe the same robot."""
import re
from pathlib import Path

import pytest

from robodog.config import load_config

PARAMS = Path(__file__).resolve().parents[2] / "cad" / "params.scad"


def scad_values():
    vals = {}
    for line in PARAMS.read_text().splitlines():
        m = re.match(r"\s*(\w+)\s*=\s*([-\d.]+)\s*;", line)
        if m:
            vals[m.group(1)] = float(m.group(2))
    return vals


def test_geometry_matches_cad():
    v, g = scad_values(), load_config()["geometry"]
    assert g["hip_x"] == v["bx"]
    assert g["l2"] == v["L2"] and g["l3"] == v["L3"]
    assert g["foot_radius"] == v["foot_d"] / 2
    # foot_y = rim_y + horn_t + link_t + horn_t + link_t/2, link_t = screw_len - horn_t
    link_t = v["screw_len"] - v["horn_t"]
    assert g["foot_y"] == pytest.approx(v["rim_y"] + 2 * v["horn_t"] + 1.5 * link_t)
