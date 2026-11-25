import math

import pytest

from robodog.config import load_config
from robodog.gait import POSE_NAMES, Velocity
from robodog.motion import MotionController
from robodog.servos import MockServoDriver


def make():
    cfg = load_config()
    drv = MockServoDriver(cfg["servos"])
    return MotionController(cfg, drv), drv


def run(mc, seconds):
    for _ in range(int(seconds / mc.dt)):
        mc.tick(mc.dt)


def joints_within_limits(mc, drv):
    """IK output must never need clamping (clamping would distort the gait)."""
    lim = drv.map.cfg
    return all(lim[n]["min"] - 1e-6 <= a <= lim[n]["max"] + 1e-6 for n, a in mc.last_joints.items())


def test_walk_cycle_reachable_and_in_range():
    mc, drv = make()
    mc.set_pose("stand"); run(mc, 2)
    for v in [(200, 0, 0), (-120, 0, 0), (0, 0, math.radians(60)), (150, 0, math.radians(30))]:
        mc.set_velocity(*v)
        run(mc, 3)
        assert joints_within_limits(mc, drv)
        assert len(drv.angles) == 10
    mc.stop(); run(mc, 2)
    assert not mc.is_moving


def test_odometry_forward():
    mc, _ = make()
    mc.set_pose("stand"); run(mc, 1)
    mc.set_velocity(100, 0, 0); run(mc, 5)
    assert 300 < mc.odometry[0] < 520


def test_obstacle_guard_blocks_forward():
    cfg = load_config()
    drv = MockServoDriver(cfg["servos"])
    mc = MotionController(cfg, drv, obstacle_distance=lambda: 100.0)
    mc.set_pose("stand"); run(mc, 1)
    mc.set_velocity(150, 0, 0); run(mc, 2)
    assert mc.blocked and abs(mc.odometry[0]) < 1


def test_all_poses_reachable():
    mc, drv = make()
    for p in POSE_NAMES:
        mc.set_pose(p); run(mc, 2)
        assert joints_within_limits(mc, drv), p


def test_head_rate_limited_and_clamped():
    mc, drv = make()
    mc.set_head(90, 40)
    mc.tick(mc.dt)
    assert mc.head[0] == pytest.approx(mc.head_rate * mc.dt)
    run(mc, 2)
    assert mc.head == [60, 25]


def test_velocity_zero():
    assert Velocity().is_zero()


def test_stop_settles_with_drift_compensation():
    """Regression: drift compensation once kept the trot running forever after stop()."""
    mc, drv = make()
    mc.set_pose("stand"); run(mc, 1)
    mc.set_velocity(150, 0, 0); run(mc, 2)
    mc.stop(); run(mc, 2)
    before = dict(mc.last_joints)
    run(mc, 0.5)
    assert all(abs(before[k] - mc.last_joints[k]) < 0.05 for k in before)   # no stepping, only pose easing
