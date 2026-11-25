"""Physics smoke test (skipped when MuJoCo or the sim meshes are absent)."""
import math
import sys
from pathlib import Path

import pytest

SIM = Path(__file__).resolve().parents[2] / "sim"
pytest.importorskip("mujoco")
pytest.importorskip("trimesh")
if not (SIM / "assets" / "tray.stl").exists():
    pytest.skip("run sim/export_meshes.sh first", allow_module_level=True)
sys.path.insert(0, str(SIM))


def test_stands_and_walks_without_falling():
    import run_sim as R
    from robodog.config import load_config

    sim = R.PipSim(load_config(), tag="pytest")
    sim.mc.set_pose("stand"); sim.step(2.5)
    t0 = sim.t
    sim.mc.set_velocity(150, 0, 0); sim.step(4)
    m = R.seg_metrics(sim, sim.arrays(), R.Segment("walk", t0 + 1.5, sim.t))
    assert not m["fell"]
    assert 100 < m["forward_speed_mm_s"] < 200
    assert m["max_roll_deg"] < 10
