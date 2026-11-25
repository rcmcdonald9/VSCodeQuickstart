import itertools
import math

import pytest

from robodog.kinematics import BodyKinematics, LegGeometry, UnreachableError, leg_fk, leg_ik

G = LegGeometry(100, 100)


def test_stand_pose_matches_cad_preview():
    # cad/assembly.scad shows pitch=30, knee=-60: foot straight under the hip
    x, z = leg_fk(30, -60, G)
    assert x == pytest.approx(0, abs=1e-9)
    assert z == pytest.approx(-100 * math.sqrt(3))
    assert leg_ik(x, z, G) == pytest.approx((30, -60))


@pytest.mark.parametrize("x,z", list(itertools.product([-70, -20, 0, 40, 70], [-110, -150, -175, -185])))
def test_ik_fk_roundtrip(x, z):
    p, k = leg_ik(x, z, G)
    assert leg_fk(p, k, G) == pytest.approx((x, z), abs=1e-6)
    assert k <= 0


def test_unreachable():
    with pytest.raises(UnreachableError):
        leg_ik(0, -205, G)


def test_symmetric_stand():
    bk = BodyKinematics(95, 57.8, G)
    j = bk.solve(bk.neutral_feet(170))
    assert all(v == pytest.approx(j["FR"]) for v in j.values())


def test_body_pitch_lengthens_rear_legs():
    bk = BodyKinematics(95, 57.8, G)
    j = bk.solve(bk.neutral_feet(170), body_rpy=(0, math.radians(8), 0))   # nose down
    # rear legs must reach further (straighter knee) than front legs
    assert j["RR"][1] > j["FR"][1]
