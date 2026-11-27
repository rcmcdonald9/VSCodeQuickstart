"""S1 bring-up + S2 teleop regression tests (no hardware).  python -m sw.test_s2

1. Protocol tables: template build/parse round trip; the DM-J4310 table is complete; the GIM6010 table lists its TODOs.
2. Bring-up, DM-J4310 path (the real tables: GIM joints BLOCKED and held in the fixture) against the virtual bench arm:
   every step passes; the zero found against the hard stop matches the plant's hidden encoder offset; the measured
   gravity residual matches the plant's hidden errors (hand mass, torque constant) computed independently.
3. Bring-up, whole arm with the SIM PLACEHOLDER GIM table (exercises the GIM code path end to end): every step passes,
   all five zeros match, collision thresholds come out for every joint and load into sw/collision.py.
4. Fault injection (each must stop the bring-up at the right step, later steps REFUSED): motor at its factory ID,
   welded contactor, dead watchdog, drive without CAN timeout, drive that ignores its torque limit, motor mounted the
   other way round.
5. Teleop (sw/teleop_sim.py): 5 scripted-leader episodes through the supervisor recorded as LeRobot v3.0 and reloaded
   with the official loader; an extra episode with an e-stop is discarded and the arm stops.
"""
from __future__ import annotations

import math
import os
import time
from pathlib import Path

import numpy as np

os.environ.setdefault("MUJOCO_GL", "egl")       # before anything imports mujoco (the teleop test renders)

OUT = Path("sw/out/test_s2")


def _quiet(*a, **k):
    pass


def test_protocol():
    from sw.drivers import protocol as P
    from sw.drivers.gim import GIM6010
    from sw.drivers.mit import DM_J4310
    tpl = DM_J4310["PARAM_WRITE"][1]
    b = P.build(tpl, 0x15, 23, 3.25, "f32")
    f = P.parse(tpl, b)
    assert f["id"] == 0x15 and f["reg"] == 23 and abs(P.decode_value(f["value_bytes"], "f32") - 3.25) < 1e-6
    assert P.arb("0x100+{id}", 5, 0x15) == 0x105 and P.arb("{fb}", 5, 0x15) == 0x15
    assert P.missing(DM_J4310) == []
    todo = P.missing(GIM6010)
    assert "CONTROL" in todo and "TORQUE_LIMIT_REG" in todo
    return f"Protocol tables: template build/parse round trip; DM-J4310 table complete; GIM6010 table has {len(todo)} TODO fields ✅"


def _bringup(**kw):
    from sw.bringup import build_virtual, load_cfg
    cfg = load_cfg()
    if kw.pop("fast", False):
        cfg["cal_speed"] = 0.5
    bu, arm = build_virtual(cfg=cfg, log=_quiet, **kw)
    t0 = time.time()
    bu.run_all()
    return bu, arm, time.time() - t0


def _offset_err(bu, arm):
    tr = arm.truth()
    return {j: abs(bu.drivers[j].offset - tr[j]["offset_joint"]) for j in bu.calibrated}


def test_bringup_dm():
    import mujoco
    from sw.bringup import ARM, STEPS
    bu, arm, dt = _bringup()
    bad = {s: bu.steps[s].status for s in STEPS if bu.steps[s].status != "PASS"}
    assert not bad, (bad, [c for s in STEPS for c in bu.steps[s].checks if not c[1]])
    assert bu.names == ["wpitch"] and set(bu.blocked) == {"pitch", "roll", "yaw", "elbow"}
    err = _offset_err(bu, arm)
    assert max(err.values()) < math.radians(0.5), err
    # gravity residual vs the plant's hidden truth: G_true / Kt_err − G_twin at the measured poses
    g = bu.steps["gravity"].data
    m = arm.env.m
    d = mujoco.MjData(m)
    kt = arm.truth()["wpitch"]["kt_err"]
    worst = 0.0
    for p in g["poses"]:
        q6 = bu.full_q(np.array(p["q"]))
        for k, a in enumerate(arm.qadr):
            d.qpos[a] = q6[k]
        mujoco.mj_forward(m, d)
        k6 = ARM.index("wpitch")
        exp = d.qfrc_bias[arm.dadr[k6]] / kt - bu.twin.gravity(q6)[k6]
        worst = max(worst, abs(p["residual"][0] - exp))
    assert worst < 0.03, worst
    mp, jp, js = bu.report(OUT, tag="_dm", bus_desc="virtual bus (test)")
    assert mp.exists() and js["calibration"]["offsets"]
    arm.close()
    return (f"Bring-up, DM-J4310 path (GIM joints BLOCKED by their TODO table, held in the fixture): all 6 steps pass in {dt:.0f} s "
            f"({bu.n_cycles} cycles at 500 Hz); zero vs the hidden encoder offset {math.degrees(max(err.values())):.3f}°; gravity residual "
            f"{g['residual_max'][0]:.3f} N·m matches the plant's hidden hand mass + Kt error within {worst * 1000:.0f} mN·m; sag test proves the drive clamp ✅")


def test_bringup_full():
    from sw.bringup import STEPS, load_collision_thresholds
    from sw.collision import CollisionDetector
    from sw.virtual_arm import SIM_GIM
    bu, arm, dt = _bringup(gim_table=SIM_GIM, fast=True, seed=3)
    bad = {s: bu.steps[s].status for s in STEPS if bu.steps[s].status != "PASS"}
    assert not bad, (bad, [c for s in STEPS for c in bu.steps[s].checks if not c[1]])
    assert bu.names == ["pitch", "roll", "yaw", "elbow", "wpitch"]
    err = _offset_err(bu, arm)
    assert len(err) == 5 and max(err.values()) < math.radians(0.5), err
    mp, jp, js = bu.report(OUT, tag="_simgim", bus_desc="virtual bus, SIM PLACEHOLDER GIM table (test)")
    thr, rel = load_collision_thresholds(jp)
    assert np.all(np.isfinite(thr[:5])) and not np.isfinite(thr[5])
    CollisionDetector(bu.twin, thr, rel)
    jog = bu.steps["jog"].data["joints"]
    arm.close()
    return (f"Bring-up, whole arm with the SIM PLACEHOLDER GIM table: all 6 steps pass in {dt:.0f} s; 5 zeros within "
            f"{math.degrees(max(err.values())):.3f}° of the hidden offsets; jog tracking ≤ {max(r['max_track_err_deg'] for r in jog):.1f}° at 10 % speed; "
            f"effective balancer K {bu.steps['gravity'].data.get('balancer_k_effective')} N·m (nominal 4.5, plant 4.725 + hidden mass/Kt); "
            f"collision thresholds {[round(float(x), 2) for x in thr[:5]]} N·m load into CollisionDetector ✅")


FAULTS = [  # (fault, step that must fail, words expected in its failed checks)
    ("factory_id:wpitch", "enumerate", "FACTORY ID"),
    ("estop_stuck", "safety", "contactor opens"),
    ("watchdog_dead", "safety", "watchdog"),
    ("no_timeout", "safety", "CAN timeout"),
    ("no_clamp", "gravity", "enforces its torque limit"),
    ("reversed:wpitch", "jog", "moved + first"),
]


def test_faults():
    from sw.bringup import STEPS
    out = []
    for fault, step, words in FAULTS:
        bu, arm, dt = _bringup(faults=[fault])
        i = STEPS.index(step)
        st = [bu.steps[s].status for s in STEPS]
        assert all(x == "PASS" for x in st[:i]) and st[i] == "FAIL" and all(x == "REFUSED" for x in st[i + 1:]), (fault, st)
        msgs = " ".join(f"{w} {d}" for w, ok, d in bu.steps[step].checks if not ok)
        assert words in msgs, (fault, msgs)
        out.append(f"{fault} → {step}")
        arm.close()
    return "Fault injection, each caught at its step with every later step REFUSED: " + "; ".join(out) + " ✅"


def test_teleop():
    from sw.teleop_sim import record, verify
    root = OUT / "teleop_v3"
    res = record(5, root, img=64, estop_episode=1, log=_quiet)
    assert res["kept"] == 5 and res["discarded"] == 1, res
    stopped = [e for e in res["episodes"] if e["stop"]]
    assert stopped[0]["stop"] == ["e-stop"] and stopped[0]["drift_after_stop_rad"] < 0.15, stopped        # torque off: it coasts (gravcomp twin)
    rl = verify(root, 5)
    assert rl["frames"] == res["frames"] and rl["codebase"] == "v3.0", (rl, res)
    return (f"Teleop (scripted leader → EEF delta → IK → supervisor at 500 Hz → MIT law on the twin): 5 episodes recorded "
            f"({res['frames']} frames, {res['success']}/5 placed, loop {res['step_ms_p50']} ms p50 / {res['step_ms_p99']} ms p99 per 15 Hz step), "
            f"reloaded with LeRobotDataset (v3.0, {rl['frames']} frames, actions exact); the e-stop episode was discarded and the arm stopped "
            f"(drift {stopped[0]['drift_after_stop_rad'] * 1000:.1f} mrad) ✅")


def main():
    out = [test_protocol(), test_bringup_dm(), test_bringup_full(), test_faults(), test_teleop()]
    Path("docs/sw").mkdir(parents=True, exist_ok=True)
    Path("docs/sw/s2_tests.md").write_text("# S1 bring-up + S2 teleop tests (generated by `python -m sw.test_s2`)\n\n"
                                           + __doc__.split("\n\n", 1)[1].strip() + "\n\n" + "\n".join(f"- {o}" for o in out) + "\n")
    print("\n".join(out))


if __name__ == "__main__":
    main()
