import itertools
import math

import pytest

from robodog.kinematics import BodyKinematics, LegGeometry, UnreachableError, leg_fk, leg_ik

G = LegGeometry(49.5, 100, 100)


def test_stand_pose_matches_cad_preview():
    # cad/assembly.scad shows roll=0, pitch=30, knee=-60 with the foot under the hip
    x, y, z = leg_fk(0, 30, -60, G)
    assert x == pytest.approx(0, abs=1e-9)
    assert y == pytest.approx(49.5)
    assert z == pytest.approx(-100 * math.sqrt(3))
    assert leg_ik(x, y, z, G) == pytest.approx((0, 30, -60))


@pytest.mark.parametrize("x,y,z", list(itertools.product([-60, 0, 60], [30, 49.5, 80], [-120, -150, -175])))
def test_ik_fk_roundtrip(x, y, z):
    angles = leg_ik(x, y, z, G)
    assert leg_fk(*angles, G) == pytest.approx((x, y, z), abs=1e-6)
    assert angles[2] <= 0  # knee always bends backward


def test_unreachable():
    with pytest.raises(UnreachableError):
        leg_ik(0, 49.5, -250, G)


def test_body_kinematics_symmetric_stand():
    bk = BodyKinematics(95, 39, G)
    joints = bk.solve(bk.neutral_feet(170))
    for leg, (r, p, k) in joints.items():
        assert r == pytest.approx(0, abs=1e-9), leg
    assert joints["FR"] == pytest.approx(joints["RL"])


def test_body_roll_lifts_one_side():
    bk = BodyKinematics(95, 39, G)
    j = bk.solve(bk.neutral_feet(170), body_rpy=(math.radians(10), 0, 0))
    # rolling the body right-side-down: left legs extend (roll outward differs in sign)
    assert j["FL"][0] != pytest.approx(j["FR"][0])
