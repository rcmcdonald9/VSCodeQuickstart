"""Bus realism on the virtual bench: bring-up, safety supervisor and collision observer against a late, lossy CAN bus.
python -m sw.bus_realism [--quick]      → docs/sw/bus_realism.md + .json

For every bus profile of sw/bus_sim.py (ideal, usb, pil9 = any delay 0–9 ms like the Asimov PIL sim, lossy, corrupt):
1. **Bring-up** (DM-J4310 path, the real protocol tables) end to end: which steps pass, which checks fail and why.
2. **Motion soak** (after its calibration): 20 s of a 0.5 Hz sine on the online joint through the supervisor at 500 Hz
   with the momentum observer (sw/collision.py) running on the measured torque, thresholds from the *ideal-bus*
   bring-up: supervisor stops and observer trips here are FALSE trips (nothing is wrong with the arm).
3. **Real faults** on the same bus: a contact (2 × the trip threshold at the joint, 0.2 s) must trip the observer; the
   bus going silent (100 % loss) must stop the supervisor (stale feedback) and the drive's own CAN timeout must
   disable it; a bus-off (100 ms) the same.
The report gives, per profile, false trips vs real-fault detections with times, the feedback-age and command-gap
distributions the loop saw, and the thresholds this implies (bringup.yaml `fb_max_age_s`, collision `rel`/`thr`).
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np

OUT = Path(__file__).resolve().parent.parent / "docs" / "sw"
PROFILE_ORDER = ["ideal", "usb", "pil9", "lossy", "corrupt"]


def _quiet(*a, **k):
    pass


def bringup(profile, seed=0, cfg=None, joints=None, gim=False, legacy=False, **kw):
    """legacy: the thresholds before this study (stale feedback 10 ms, no frame plausibility gate)."""
    from sw.bringup import STEPS, build_virtual, load_cfg
    from sw.virtual_arm import SIM_GIM
    cfg = cfg or load_cfg()
    cfg["cal_speed"] = 0.5
    if legacy:
        cfg["fb_max_age_s"] = 0.010
    bu, arm = build_virtual(cfg=cfg, log=_quiet, bus=profile, seed=seed, joints=joints, gim_table=SIM_GIM if gim else None, **kw)
    if legacy:
        orig = bu.step_enumerate

        def enum():
            orig()
            for d in bu.drivers.values():
                d.gate = False
        bu.step_enumerate = enum
    t0 = time.time()
    bu.run_all()
    res = dict(profile=profile if isinstance(profile, str) else "custom", seed=seed, wall_s=round(time.time() - t0, 1),
               steps={s: bu.steps[s].status for s in STEPS},
               failed=[f"{s}: {w} — {d}" for s in STEPS for w, ok, d in bu.steps[s].checks if not ok],
               bus=arm.bus_stats())
    ct = bu.steps["gravity"].data.get("collision_thresholds")
    res["thr"] = ct["thr"] if ct else None
    return bu, arm, res


def motion(bu, arm, thr, rel=0.10, secs=20.0, amp_deg=20.0, f_hz=0.5):
    """Sine on every online joint through the supervisor, observer in observe mode. → stats (false trips)."""
    from sw.bringup import ARM, D2R
    from sw.collision import CollisionDetector
    if bu.io.estop_active():
        bu.arm()
    sup = bu.supervisor(v_max=np.array([1.5 * amp_deg * D2R * 2 * math.pi * f_hz] * len(bu.names)))
    bu.start(sup)
    bu.enable_all(sup)
    th = np.array([np.inf if t is None else t for t in thr])
    det = CollisionDetector(bu.twin, th, np.where(np.isfinite(th), rel, 0.0))
    centre = {j: bu.jc(j)["park"] * D2R for j in bu.names}
    bu._move(sup, centre, settle=0.2)
    n = int(secs / bu.dt)
    ages, gaps, trips, stops = [], [], [], []
    last_fresh = bu.now()
    w = 2 * math.pi * f_hz
    t0 = bu.now()
    peak = 0.0
    for i in range(n):
        t = (i + 1) * bu.dt
        ramp = min(1.0, t / 1.0)
        for j in bu.names:
            A = amp_deg * D2R * ramp
            bu.hold_q[j] = centre[j] + A * math.sin(w * t)
            bu.hold_v[j] = A * w * math.cos(w * t)
            bu.hold_a[j] = -A * w * w * math.sin(w * t)
        v, fb = bu.cycle(sup, *bu._cmd_hold(bu.feedback()))
        fresh = any(d.state.get("fresh") for d in bu.drivers.values())
        if fresh:
            gaps.append(bu.now() - last_fresh)
            last_fresh = bu.now()
        if fb.age is not None:
            ages.append(fb.age)
        if v.stop:
            stops.append(dict(t=round(bu.now() - t0, 3), reasons=list(v.reasons)))
            break
        q6, v6, t6 = bu._six(fb)
        if det.update(q6, v6, t6, bu.dt) and not trips:
            trips.append(dict(t=round(bu.now() - t0, 3), joint=ARM[det.joint]))
            det.reset()
        peak = max(peak, det.peak_ratio)
    A = np.array(ages) * 1000 if ages else np.zeros(1)
    G = np.array(gaps) * 1000 if gaps else np.zeros(1)
    return dict(cycles=i + 1, supervisor_stops=stops, observer_trips=trips, observer_peak_ratio=round(peak, 3),
                fb_age_ms=dict(p50=round(float(np.percentile(A, 50)), 2), p99=round(float(np.percentile(A, 99)), 2),
                               p999=round(float(np.percentile(A, 99.9)), 2), max=round(float(A.max()), 2)),
                fresh_gap_ms=dict(p99=round(float(np.percentile(G, 99)), 2), max=round(float(G.max()), 2)),
                rejected_frames=sum(getattr(d, "rejected", 0) for d in bu.drivers.values()), sup=sup, det=det)


def real_faults(bu, arm, sup, det, thr):
    """Contact, bus silent, bus-off: each must be detected. Times in ms from the fault."""
    from sw.bringup import ARM
    out = {}
    j = bu.names[0]
    k6 = ARM.index(j)
    det.reset()
    t_c = bu.now()
    arm.ext_tau[k6] = 2.0 * float(thr[k6])
    t_trip = None
    while bu.now() - t_c < 0.2:
        v, fb = bu.cycle(sup, *bu._cmd_hold(bu.feedback()))
        if v.stop:
            break
        q6, v6, t6 = bu._six(fb)
        if det.update(q6, v6, t6, bu.dt) and t_trip is None:
            t_trip = bu.now() - t_c
    arm.ext_tau[k6] = 0.0
    out["contact_trip_ms"] = None if t_trip is None else round(t_trip * 1000, 1)
    for _ in range(100):                                       # let it settle
        v, fb = bu.cycle(sup, *bu._cmd_hold(bu.feedback()))
        if v.stop:
            break
    for kind in ("silent", "bus_off"):
        if sup.latched or bu.io.estop_active():
            if bu.io.estop_active():
                bu.arm()
            bu.clear_errors()                                  # the drives timed out on the last fault
            bu.disable_all()
            sup = bu.supervisor()
            bu.start(sup)
            bu.enable_all(sup)
        imp = [arm.imp[bn] for bn in arm.imp]
        saved = [(i.p_drop, i.ideal) for i in imp]
        t_f = bu.now()
        for i in imp:
            if kind == "silent":
                i.p_drop, i.ideal = 1.0, False
            else:
                i.ideal = False
                i.bus_off_at, i.bus_off_ms = t_f, 100.0
        t_stop = t_dis = None
        reason = None
        while bu.now() - t_f < 0.25:
            v, fb = bu.cycle(sup, *bu._cmd_hold(bu.feedback()))
            if v.stop and t_stop is None:
                t_stop, reason = bu.now() - t_f, list(v.reasons)
            if t_dis is None and not any(mt.enabled for mt in arm.motors):
                t_dis = bu.now() - t_f
            if t_stop is not None and t_dis is not None:
                break
        for i, (pd, idl) in zip(imp, saved):
            i.p_drop, i.ideal = pd, idl
        out[kind] = dict(supervisor_stop_ms=None if t_stop is None else round(t_stop * 1000, 1), reason=reason,
                         drive_disabled_ms=None if t_dis is None else round(t_dis * 1000, 1),
                         adapter=bu.buses[arm.motors[0].bus].error_counters())
        for _ in range(60):                                    # bus back: let frames flow again
            bu.refresh_cycle(heartbeat=True)
    return out


def run(profiles=PROFILE_ORDER, seed=0, secs=20.0, log=print, legacy=False):
    from sw.bringup import load_cfg
    cfg = load_cfg()
    res = {}
    base_thr = None
    for p in profiles:
        bu, arm, r = bringup(p, seed, cfg=dict(cfg), legacy=legacy)
        if p == "ideal":
            base_thr = r["thr"]
        if bu.calibrated:
            m = motion(bu, arm, base_thr, secs=secs)
            sup, det = m.pop("sup"), m.pop("det")
            r["motion"] = m
            if not m["supervisor_stops"]:
                r["faults"] = real_faults(bu, arm, sup, det, base_thr)
        r["bus_after"] = arm.bus_stats()
        arm.close()
        res[p] = r
        log(f"{p:8s} steps {''.join(s[0] for s in r['steps'].values())}  failed {len(r['failed'])}  "
            f"motion {r.get('motion', {}).get('supervisor_stops')} trips {r.get('motion', {}).get('observer_trips')} "
            f"faults {r.get('faults')}")
    return dict(date=time.strftime("%Y-%m-%d"), seed=seed, motion_s=secs, base_thr=base_thr, profiles=res, legacy=legacy)


def write(R, name="bus_realism"):
    from sw import bus_sim
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"{name}.json").write_text(json.dumps(R, indent=1, default=str))
    L = ["# Bus realism on the virtual bench (generated by `python -m sw.bus_realism`)", "",
         f"{R['date']}, seed {R['seed']}{' — **LEGACY thresholds** (stale feedback 10 ms, no frame plausibility gate)' if R.get('legacy') else ''}: bring-up on the DM-J4310 path (real protocol tables), then {R['motion_s']:.0f} s of a 0.5 Hz ±20° "
         "sine through the supervisor with the collision observer on (thresholds from the ideal-bus bring-up), then real faults. "
         "Profiles: sw/bus_sim.py.", "",
         "| Profile | Spec | Bring-up steps | Failed checks | Motion: supervisor stops | Observer false trips (peak / threshold) | fb age p99.9 / max ms | Contact trip | Bus silent → supervisor stop / drive off | Bus-off → stop / drive off |",
         "|---|---|---|---|---|---|---|---|---|---|"]
    for p, r in R["profiles"].items():
        m, f = r.get("motion", {}), r.get("faults", {})
        sil, bo = f.get("silent", {}), f.get("bus_off", {})
        spec = bus_sim.PROFILES.get(p, {})
        L.append(f"| {p} | {', '.join(f'{k}={v}' for k, v in spec.items()) or '—'} | {' '.join(f'{k[:4]} {v}' for k, v in r['steps'].items())} | "
                 f"{'; '.join(r['failed'])[:300] or '—'} | {m.get('supervisor_stops') or 0} | {len(m.get('observer_trips') or [])} ({m.get('observer_peak_ratio')}) | "
                 f"{m.get('fb_age_ms', {}).get('p999')} / {m.get('fb_age_ms', {}).get('max')} | {f.get('contact_trip_ms')} ms | "
                 f"{sil.get('supervisor_stop_ms')} / {sil.get('drive_disabled_ms')} ms | {bo.get('supervisor_stop_ms')} / {bo.get('drive_disabled_ms')} ms |")
    L += ["", "Round-trip command → reply as the plant delivered it, and the CAN error counters the host adapter reported:", "",
          "| Profile | round trip mean / p95 / max ms | frames dropped / duplicated / corrupted / CRC (both directions) | adapter state after the faults |", "|---|---|---|---|"]
    for p, r in R["profiles"].items():
        b = r["bus_after"]
        rt = b["round_trip_ms"]
        tot = {k: sum(b["buses"][bn][d][k] for bn in b["buses"] for d in ("tx", "rx")) for k in ("dropped", "duplicated", "corrupted", "crc_errors")}
        st = {bn: v["errors"]["state"] + f" (bus-off {v['errors']['bus_off']}, restarts {v['errors']['restarts']})" for bn, v in b["buses"].items()}
        L.append(f"| {p} | {rt['mean']} / {rt['p95']} / {rt['max']} | {tot['dropped']} / {tot['duplicated']} / {tot['corrupted']} / {tot['crc_errors']} | {st} |")
    if R.get("notes"):
        L += ["", "## Findings and fixes", ""] + R["notes"]
    (OUT / f"{name}.md").write_text("\n".join(L) + "\n")
    return L


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--quick", action="store_true", help="5 s of motion instead of 20")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--profiles", default=",".join(PROFILE_ORDER))
    ap.add_argument("--name", default="bus_realism")
    ap.add_argument("--legacy", action="store_true", help="the thresholds before this study: stale feedback 10 ms, no plausibility gate")
    a = ap.parse_args(argv)
    R = run(a.profiles.split(","), a.seed, 5.0 if a.quick else 20.0, legacy=a.legacy)
    print("\n".join(write(R, a.name)))


if __name__ == "__main__":
    main()
