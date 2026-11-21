import math

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
    lo, hi = 0.0, drv.map.range
    return all(v is None or lo <= v <= hi for v in drv.last.values())


def test_walk_cycle_reachable_and_in_range():
    mc, drv = make()
    mc.set_pose("stand"); run(mc, 2)
    for v in [(180, 0, 0), (-120, 0, 0), (0, 100, 0), (0, 0, math.radians(50)), (150, 0, math.radians(30))]:
        mc.set_velocity(*v)
        run(mc, 3)
        assert joints_within_limits(mc, drv)
        assert len(drv.last) == 12
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


def test_velocity_zero():
    assert Velocity().is_zero()
