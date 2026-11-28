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
5. Bus realism (sw/bus_sim.py): 0–9 ms delay each way, drops, duplicates, CRC errors, bus-off, corrupted payloads: the
   old 10 ms stale-feedback bound false-trips; the tuned bring-up passes, no false trip in motion, real faults (contact,
   silent bus, bus-off) still caught; corrupted frames are dropped by the driver's plausibility gate.
6. Actuator sysid on the Step-1 rig with hidden friction / torque constant / backlash and a USB-CAN bus: the fit recovers
   them within tolerance; actuators_<serial>.yaml is loaded back by rnd/twin.py and sw/virtual_arm.py.
7. Soak test: sysid before and after cyclic motion; a stable drive passes, a drive whose friction drifts fails.
8. Teleop (sw/teleop_sim.py): 5 scripted-leader episodes through the supervisor recorded as LeRobot v3.0 and reloaded
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
    md = mp.read_text()
    assert "## Determinism" in md and "## Fitted actuator model" in md and js["determinism"]["verdict"], "report sections"
    assert bu.steps["sysid"].data["fits"]["wpitch"]["kt_scale"] is not None
    arm.close()
    return (f"Bring-up, DM-J4310 path (GIM joints BLOCKED by their TODO table, held in the fixture): all 7 steps pass in {dt:.0f} s "
            f"({bu.n_cycles} cycles at 500 Hz); zero vs the hidden encoder offset {math.degrees(max(err.values())):.3f}°; gravity residual "
            f"{g['residual_max'][0]:.3f} N·m matches the plant's hidden hand mass + Kt error within {worst * 1000:.0f} mN·m; sag test proves the drive clamp; "
            f"sysid K {bu.steps['sysid'].data['fits']['wpitch']['kt_scale']:.3f} (hidden {kt:.3f}); report has the fitted model and the determinism checks ✅")


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
    return (f"Bring-up, whole arm with the SIM PLACEHOLDER GIM table: all 7 steps pass in {dt:.0f} s; 5 zeros within "
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


def test_bus_realism():
    """Late / lossy / corrupt CAN (sw/bus_sim.py) against the bring-up, the supervisor and the collision observer."""
    import numpy as np
    from sw import bus_realism as BR
    from sw.bringup import STEPS, load_cfg
    # a. the old 10 ms stale-feedback bound false-trips on a 0–9 ms-each-way bus (the arm is fine)
    cfg = load_cfg()
    bu, arm, r = BR.bringup("pil9", 0, cfg=cfg, legacy=True)
    assert r["steps"]["safety"] == "FAIL" and any("stale" in f for f in r["failed"]), r["failed"]
    arm.close()
    # b. tuned (20 ms, plausibility gate, fresh-after-request, CLEAR_ERROR confirmed): every step passes on pil9, no false
    #    trip in motion, and the real faults are still caught
    bu, arm, r = BR.bringup("pil9", 0, cfg=load_cfg())
    assert all(v == "PASS" for v in r["steps"].values()), (r["steps"], r["failed"])
    thr = [None, None, None, None, 0.3, None]
    m = BR.motion(bu, arm, thr, secs=4.0)
    sup, det = m.pop("sup"), m.pop("det")
    assert not m["supervisor_stops"] and not m["observer_trips"], m
    f = BR.real_faults(bu, arm, sup, det, np.array([np.inf] * 4 + [0.3, np.inf]))
    assert f["contact_trip_ms"] is not None and f["contact_trip_ms"] <= 25, f
    assert f["silent"]["supervisor_stop_ms"] <= 30 and f["silent"]["drive_disabled_ms"] <= 40, f
    assert f["bus_off"]["supervisor_stop_ms"] <= 35 and f["bus_off"]["adapter"]["bus_off"] >= 1, f
    rt = arm.bus_stats()["round_trip_ms"]
    arm.close()
    # c. corrupted payloads (a buggy controller driver): one flipped position bit used to stop calibrate ("tracking
    #    error"); the gate now drops implausible frames and the bring-up passes
    bu, arm, r3 = BR.bringup("corrupt", 0, cfg=load_cfg())
    rej = sum(d.rejected for d in bu.drivers.values())
    assert all(v == "PASS" for v in r3["steps"].values()), (r3["steps"], r3["failed"])
    corrupted = sum(arm.imp[bn].c[dd].corrupted for bn in arm.imp for dd in ("tx", "rx"))
    assert corrupted > 0 and rej > 0, (corrupted, rej)
    arm.close()
    return (f"Bus realism (sw/bus_sim.py; docs/sw/bus_realism.md): with 0–9 ms of delay each way (round trip mean {rt['mean']:.0f}, max {rt['max']:.0f} ms) "
            f"the old 10 ms stale-feedback bound false-trips the safety step; with 20 ms + the fixes every step of the bring-up passes, "
            f"no false stop or collision trip in motion, a contact still trips in {f['contact_trip_ms']:.0f} ms, a silent bus stops the "
            f"supervisor in {f['silent']['supervisor_stop_ms']:.0f} ms and the drive's own timeout fires at {f['silent']['drive_disabled_ms']:.0f} ms, bus-off "
            f"in {f['bus_off']['supervisor_stop_ms']:.0f} ms; {corrupted} corrupted payloads → {rej} dropped by the plausibility gate, bring-up passes ✅")


HARD_FAULTS = ["e-stop", "process hang", "NaN feedback", "bus silent", "bus-off", "over-temperature", "supply trip"]


def test_integrated_stop():
    """Astra review #3, critical 4: the production path detector → supervisor → heartbeat → safety board on the virtual
    bench. (i) A contact latches the collision fault only: compliant hold with the heartbeat running and the contactor
    closed. (ii) A hard fault during that hold (e-stop, hung loop, NaN feedback, silent bus, bus-off, over-temperature, a
    supply trip) drops the heartbeat within its detection budget, overrides the hold (stop, not compliant), opens the
    contactor and removes drive torque within 50 ms; nothing re-arms until the operator does. (iii) Heartbeat gaps (sim
    time: cycles where a healthy loop would not toggle the line) and the host's per-cycle compute (wall clock) under every
    bus-realism profile with the observer in the loop. The board is set to its worst case: monostable +20 %, 10 ms release."""
    import json
    from sw import bus_sim
    from sw.bringup import ARM, STEPS, build_virtual, load_cfg
    from sw.bus_realism import PROFILE_ORDER
    from sw.safety import TEMP_C
    from rnd import electrical as el
    cfg = load_cfg()
    cfg["cal_speed"] = 0.5
    bu, arm = build_virtual(cfg=cfg, log=_quiet)
    bu.run_all(until="gravity")
    bad = {s: bu.steps[s].status for s in STEPS[:STEPS.index("gravity") + 1] if bu.steps[s].status != "PASS"}
    assert not bad, bad
    io = arm.io
    wd = float(cfg["watchdog_s"])
    io.WD_S, io.t_release = wd * (1 + el.WATCHDOG["tol"]), el.CONTACTOR["t_release"]
    sim = json.loads((Path("docs/rnd/safety_board_sim.json")).read_text()) if Path("docs/rnd/safety_board_sim.json").exists() else {}
    t_tail = next((d["t_uvlo_far"] for d in sim.get("discharge", []) if d["which"] == "M1"), None) or el.stop_timing("M1")["rows"][3][1]
    t_tail += 0.001                                          # board: aux → FORCE → far drive < UVLO; then the phase current (electrical.md)
    j0 = bu.names[0]
    k6 = ARM.index(j0)
    ct = bu.steps["gravity"].data["collision_thresholds"]
    thr6 = float(ct["thr"][k6])
    online = [m for m in arm.motors if m.name in bu.names]
    dt = bu.dt

    def torque_on():
        return any(m.powered and m.enabled for m in online)

    def session(n_hold=150):
        if io.button:
            io.release()
        if io.estop_active():
            bu.arm()                                         # the operator re-arms (raises StepFail if it cannot)
        bu.clear_errors()
        bu.disable_all()
        sup = bu.supervisor(collision=bu.collision())
        bu.start(sup)
        bu.enable_all(sup)
        for _ in range(n_hold):
            v, fb = bu.cycle(sup, *bu._cmd_hold(bu.feedback()))
            assert not v.stop and not v.compliant, v.reasons
        return sup

    def contact(sup, hold_s=1.0):
        """2 × the threshold at the joint for 0.2 s, then the compliant hold for hold_s: heartbeat every cycle, contactor closed."""
        arm.ext_tau[k6] = 2.0 * thr6
        t0, t_trip = bu.now(), None
        hb0 = len(bu.hb_gaps)
        n = int((0.2 + hold_s) / dt)
        for i in range(n):
            if i == int(0.2 / dt):
                arm.ext_tau[k6] = 0.0
            v, fb = bu.cycle(sup, *bu._cmd_hold(bu.feedback()))
            assert not v.stop, ("collision must not stop", v.reasons)
            if v.compliant and t_trip is None:
                t_trip = bu.now() - t0
            if t_trip is not None:
                assert v.compliant and sup.latched == ["collision"], (sup.latched, v.reasons)
                assert sup.heartbeat_ok() and io.closed and torque_on(), "compliant hold must keep the heartbeat, the contactor and the drives"
        assert t_trip is not None and t_trip <= 0.025, t_trip
        return t_trip, max(bu.hb_gaps[hb0:]) if len(bu.hb_gaps) > hb0 else None

    # (i) + (ii): a contact, then each hard fault during the compliant hold
    rows = []
    orig_fb = bu.feedback
    for kind in HARD_FAULTS:
        sup = session()
        t_trip, gap_hold = contact(sup)
        saved = [(arm.imp[bn].p_drop, arm.imp[bn].ideal) for bn in arm.imp]
        t_f = bu.now()
        t_hb_last = bu.t_hb
        t_drop = t_override = t_torque = None
        if kind == "e-stop":
            bu.op.press_estop()
        elif kind == "NaN feedback":
            def nan_fb():
                f = orig_fb()
                f.q[0] = np.nan
                return f
            bu.feedback = nan_fb
        elif kind == "bus silent":
            for i in arm.imp.values():
                i.p_drop, i.ideal = 1.0, False
        elif kind == "bus-off":
            for i in arm.imp.values():
                i.ideal, i.bus_off_at, i.bus_off_ms = False, t_f, 100.0
        elif kind == "over-temperature":
            for m in online:
                m.temp = TEMP_C + 15.0
        elif kind == "supply trip":
            io.drop()
        t_end = t_f + 0.2
        while bu.now() < t_end:
            if kind == "process hang":
                arm.step(dt)                                 # the host is silent: no cycle, no heartbeat, no command
                if bu.now() - t_f > 0.1:
                    break
            else:
                v, fb = bu.cycle(sup, *bu._cmd_hold(bu.feedback()))
                if t_drop is None and not sup.heartbeat_ok():
                    t_drop = bu.now() - t_f
                if t_override is None and v.stop:
                    assert not v.compliant
                    t_override = bu.now() - t_f
            if t_torque is None and not torque_on():
                t_torque = bu.now() - t_f
        if kind == "process hang":
            t_drop = t_override = 0.0
        t_open = None if io.t_open is None or io.t_open < t_f else io.t_open - t_f
        assert t_drop is not None and t_override is not None, (kind, t_drop, t_override, sup.latched)
        assert t_open is not None and not io.closed, (kind, "contactor did not open")
        assert t_torque is not None and t_torque <= el.SW2_BUDGET, (kind, t_torque)
        hw_end = t_open + t_tail                              # contacts open → bus below UVLO at the far drive (simulated board)
        # clear the fault: nothing may re-arm by itself
        bu.feedback = orig_fb
        for i, (pd, idl) in zip(arm.imp.values(), saved):
            i.p_drop, i.ideal, i.bus_off_at = pd, idl, None
        for m in online:
            m.temp = 32.0
        if kind == "e-stop":
            bu.op.release_estop()
        if kind != "process hang":
            for _ in range(100):
                v, fb = bu.cycle(sup, *bu._cmd_hold(bu.feedback()))
                assert v.stop and not sup.heartbeat_ok() and not io.closed, (kind, "self re-arm", sup.latched)
        else:
            for _ in range(100):
                bu.refresh_cycle()
            assert not io.closed, "hang: self re-arm"
        rows.append(dict(fault=kind, collision_trip_ms=round(t_trip * 1e3, 1), hold_gap_max_ms=round(gap_hold * 1e3, 2) if gap_hold else None,
                         hb_drop_ms=round(t_drop * 1e3, 1), override_ms=round(t_override * 1e3, 1), contactor_open_ms=round(t_open * 1e3, 1),
                         torque_off_ms=round(t_torque * 1e3, 1), hw_no_torque_ms=round(min(hw_end, el.SW2_BUDGET * 9) * 1e3, 1),
                         latched=list(sup.latched)))
        assert min(t_torque, hw_end) <= el.SW2_BUDGET, rows[-1]
    # (iii) heartbeat gaps + host compute per bus profile, observer in the loop
    from time import perf_counter
    plant_t = [0.0]
    step0 = arm.step

    def timed_step(d):
        t = perf_counter()
        step0(d)
        plant_t[0] += perf_counter() - t
    arm.step = timed_step
    gaps = {}
    for prof in PROFILE_ORDER:
        for bn in arm.imp:
            arm.imp[bn] = bus_sim.make(prof, np.random.default_rng(11), bn)
            bu.buses[bn].err_source = arm.imp[bn].error_counters
        sup = session(n_hold=50)
        hb0 = len(bu.hb_gaps)
        centre = bu.hold_q[j0]
        host, stops, trips = [], 0, 0
        for i in range(int(3.0 / dt)):                        # 3 s of a 0.5 Hz ±20° sine, then a contact and 0.5 s of compliant hold
            t = (i + 1) * dt
            if t < 2.5:
                w = 2 * math.pi * 0.5
                A = math.radians(20) * min(1.0, t)
                bu.hold_q[j0], bu.hold_v[j0], bu.hold_a[j0] = centre + A * math.sin(w * t), A * w * math.cos(w * t), -A * w * w * math.sin(w * t)
            else:
                bu.hold_q[j0], bu.hold_v[j0], bu.hold_a[j0] = bu.hold_q[j0], 0.0, 0.0
            arm.ext_tau[k6] = 2.0 * thr6 if 2.5 <= t < 2.6 else 0.0
            p0, c0 = plant_t[0], perf_counter()
            v, fb = bu.cycle(sup, *bu._cmd_hold(bu.feedback()))
            host.append((perf_counter() - c0) - (plant_t[0] - p0))
            stops += int(v.stop)
            trips += int(v.compliant and t < 2.5)
            if v.stop:
                break
        g = np.array(bu.hb_gaps[hb0:]) * 1e3
        H = np.array(host) * 1e3
        gaps[prof] = dict(cycles=len(H), hb_gap_max_ms=round(float(g.max()), 2), hb_gap_p999_ms=round(float(np.percentile(g, 99.9)), 2),
                          supervisor_stops=stops, false_collision_trips=trips, compliant_at_end=bool(v.compliant),
                          host_ms=dict(p50=round(float(np.percentile(H, 50)), 3), p999=round(float(np.percentile(H, 99.9)), 3), max=round(float(H.max()), 3)))
        assert stops == 0 and v.compliant, (prof, gaps[prof])
        assert g.max() <= 0.5 * wd * 1e3, (prof, gaps[prof])
        if prof in ("ideal", "usb"):
            assert trips == 0, (prof, gaps[prof])
    arm.step = step0
    arm.close()
    out = dict(date=time.strftime("%Y-%m-%d"), watchdog_ms=wd * 1e3, board=dict(wd_ms=io.WD_S * 1e3, release_ms=io.t_release * 1e3, tail_ms=round(t_tail * 1e3, 2)),
               faults=rows, profiles=gaps, joint=j0)
    Path("docs/sw").mkdir(parents=True, exist_ok=True)
    Path("docs/sw/integrated_stop.json").write_text(json.dumps(out, indent=1))
    worst_hold = max(r["hold_gap_max_ms"] or 0 for r in rows)
    fr = "; ".join(f"{r['fault']} {r['hb_drop_ms']:.0f}/{r['contactor_open_ms']:.0f}/{min(r['torque_off_ms'], r['hw_no_torque_ms']):.0f}" for r in rows)
    pg = "; ".join(f"{p} {v['hb_gap_max_ms']:.1f} ms (host compute p99.9 {v['host_ms']['p999']:.2f} / max {v['host_ms']['max']:.1f} ms)" for p, v in gaps.items())
    return (f"Integrated stop (Astra #3 critical 4; detector → supervisor → heartbeat → SimSafetyIO at monostable +20 % and a 10 ms release, {j0}; "
            f"docs/sw/integrated_stop.json): a contact (2 × {thr6:.2f} N·m) latches the collision fault only and the arm holds compliantly for 1 s with the heartbeat "
            f"every cycle (longest gap {worst_hold:.1f} ms) and the contactor closed; each hard fault then overrides the hold, ms to heartbeat drop / contactor open / "
            f"no drive torque: {fr} (≤ 50 ms); none re-arms until the operator does. Heartbeat gaps with the observer in the loop, 3 s motion + contact + hold, "
            f"longest per bus profile: {pg} (watchdog {wd * 1e3:.0f} ms) ✅")


def test_sysid():
    from sw.rehearse_step1 import TOL_PCT, rehearse, write
    bu, arm, res = rehearse("usb", 0, log=_quiet)
    assert all(v == "PASS" for v in res["steps"].values()), (res["steps"], res["failed"])
    worst = {}
    for j, t in res["truth_vs_fit"].items():
        for q, v in t.items():
            assert v["err_pct"] is not None and abs(v["err_pct"]) <= TOL_PCT[q], (j, q, v)
            worst[q] = max(worst.get(q, 0.0), abs(v["err_pct"]))
    write(res)
    # real2sim: a new virtual arm loads the fitted files instead of its defaults
    from sw.bringup import build_virtual, load_cfg
    fdir = OUT.parent / "rehearse_step1" / "actuators"
    from rnd.twin import actuator_model, measured_actuators
    fits = measured_actuators(fdir)
    assert set(fits) == {"yaw", "wpitch"}, fits
    assert actuator_model("wpitch", fdir)["source"].startswith("fitted") and actuator_model("wpitch")["source"].startswith("datasheet")
    bu2, arm2 = build_virtual(cfg=load_cfg(), log=_quiet, actuator_dir=fdir)
    assert arm2.act_source.startswith("fitted") and abs(arm2.act["wpitch"]["coulomb_nm"] - fits["wpitch"]["coulomb_nm"]) < 1e-9
    arm.close()
    arm2.close()
    return ("Actuator sysid (bring-up step 7, sw/sysid.py) rehearsed on the Step-1 rig (GIM yaw stage on a lever + DM-J4310, hidden values, "
            "USB-CAN bus): fit vs hidden truth worst error " + ", ".join(f"{q} {v:.1f} %" for q, v in worst.items())
            + " (docs/sw/sysid_rehearsal.md); actuators_<serial>.yaml written and loaded back by rnd/twin.py and sw/virtual_arm.py ✅")


def test_soak():
    from sw import soak
    from sw.bringup import build_virtual, load_cfg
    out = {}
    for fault in ((), ("drift",)):
        cfg = load_cfg()
        cfg["cal_speed"] = 0.5
        cfg["soak"] = dict(mass_kg=0.5, sample_s=10.0)
        cfg["sysid"] = dict(latency_steps=12)
        bu, arm = build_virtual(cfg=cfg, faults=fault, bus="usb", log=_quiet, actuator_dir=OUT / "soak", act={})
        res = soak.run(bu, 45.0, log=_quiet)
        soak.write(res, "soak_virtual" + ("_drift" if fault else ""))
        out[fault] = res
        arm.close()
    ok, bad = out[()], out[("drift",)]
    assert ok["status"] == "PASS", ok.get("fails")
    assert bad["status"] == "FAIL" and any("coulomb" in f for f in bad["fails"]), bad.get("fails")
    d0 = ok["drift"]["wpitch"]["drift"]
    return (f"Soak test (sw/soak.py, Step-1 exit criterion): sysid at start and end of cyclic motion; stable virtual drive PASS (drift Coulomb "
            f"{d0['coulomb_pct']:+.1f} %, K {d0['kt_pct']:+.2f} %, latency {d0['latency_mean_ms']:+.1f} ms); a drive whose friction grows with temperature "
            f"FAILS ({'; '.join(bad['fails'])}) ✅")


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


def test_gim_discover():
    from sw.gim_discover import rehearse
    rep = rehearse(enable=True, log=_quiet)
    assert rep["mit_plausible"] and rep["phases"]["enable"]["p_span"] > 0.1 and rep["phases"]["enable"]["t_max_seen"] < 0.5, rep
    return ("GIM protocol discovery (bring-up step 1, sw/gim_discover.py) against a simulated MIT-family drive: listen, no-torque "
            f"exit-motor-mode probe answered on ID {rep['phases']['enable']['node']}, zero-gain motor mode tracks a hand turn "
            f"({rep['phases']['enable']['p_span']:.2f} rad, |τ| ≤ {rep['phases']['enable']['t_max_seen']:.2f} N·m) ✅")


def main():
    out = [test_protocol(), test_gim_discover(), test_bringup_dm(), test_bringup_full(), test_faults(), test_bus_realism(), test_integrated_stop(), test_sysid(),
           test_soak(), test_teleop()]
    Path("docs/sw").mkdir(parents=True, exist_ok=True)
    Path("docs/sw/s2_tests.md").write_text("# S1 bring-up + S2 teleop tests (generated by `python -m sw.test_s2`)\n\n"
                                           + __doc__.split("\n\n", 1)[1].strip() + "\n\n" + "\n".join(f"- {o}" for o in out) + "\n")
    print("\n".join(out))


if __name__ == "__main__":
    main()
