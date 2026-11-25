import pytest

from robodog.config import load_config
from robodog.servos import JointMap, MockServoDriver, joint_names


def test_all_joints_configured():
    cfg = load_config()
    assert set(cfg["servos"]["joints"]) == set(joint_names())
    ids = [j["id"] for j in cfg["servos"]["joints"].values()]
    assert len(ids) == len(set(ids)) == 10


def test_counts_roundtrip_and_left_right_mirror():
    cfg = load_config()
    m = JointMap(cfg["servos"])
    for name in joint_names():
        for a in (-20.0, 0.0, 17.5):
            a = m.clamp(name, a)
            assert m.to_angle(name, m.to_counts(name, a)) == pytest.approx(a, abs=0.1)
    # identical modules, rotated 180°: the same joint angle turns left/right servos opposite ways
    assert (m.to_counts("FR.pitch", 10) - 2048) == -(m.to_counts("FL.pitch", 10) - 2048)


def test_limits_clamp():
    cfg = load_config()
    d = MockServoDriver(cfg["servos"])
    d.write({"FR.pitch": 90, "head.tilt": -80})
    assert d.angles["FR.pitch"] == 50 and d.angles["head.tilt"] == -25
