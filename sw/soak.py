"""Soak test: hour 20 must behave like hour 1 (Asimov sim2real lesson 1; Step-1 exit criterion, docs/M1_BENCH_ARM.md).
python -m sw.soak --rig step1 --bus socketcan:can0,can1 --estop gpiod:gpiochip0:17:27 --joints yaw,wpitch   (Step-1 rig, 2 h)
python -m sw.soak --rig step1 --bus virtual [--seconds 60] [--fault drift]                                 (rehearsal)

Definition (SOAK below):
1. bring-up enumerate → calibrate (the zero; limits and the safety chain re-checked);
2. **sysid fit at the start** (sw/sysid.py: friction, torque constant, backlash, latency) with the drives cold;
3. **2 h cyclic motion at ≈ 50–60 % rated torque**: a 0.25 Hz sine (± the jog amplitude) on every joint around the angle
   where gravity loads it most, with the soak weight on. **Step-1 rig** (`--rig step1`, docs/M1_RIG.md): discs bolted
   through each lever at 0.27 m, the GIM6010-8 lever 4 discs (1.04 kg with the carrier: 3.0 N·m horizontal, ≈ 55 % of its
   5 N·m RMS over the sine), the DM-J4310 lever 2 discs (0.60 kg: 1.86 N·m, ≈ 57 % of 3 N·m RMS); the run FAILS if a
   joint's measured RMS torque leaves the profile's `load_band` (45–65 %). (The old "2 kg at 0.3 m ≈ 59 %" assumed the
   arm's 2:1 yaw stage: on the bare motor it was 118 % of rated, Fable review 2026-11-28.) The arm (Step 2): the
   bringup.yaml soak weight at the grasp. The supervisor in every cycle; every 60 s a determinism sample (CPU
   temperature, throttle flags, CAN error counters, drive temperatures);
4. **sysid fit at the end**, drives hot;
5. PASS if every drift is inside LIMITS and the run was clean: no supervisor stop, no bus-off, no new throttle flag,
   no implausible (corrupted) feedback frame, loop jitter p99 < 1 ms (on the Pi), CPU < 80 °C, and (when the profile
   gives one) every joint's RMS torque inside the load band.
Limits are what the simulator absorbs without randomising (sw/sysid.py NOT_RANDOMIZED): beyond them the dependence
(e.g. on temperature) goes into the actuator model, or the hardware is fixed; the range is not widened.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np

OUT = Path(__file__).resolve().parent.parent / "docs" / "sw"
SOAK = dict(hours=2.0, load_frac=0.5, freq_hz=0.25, sample_s=60.0, mass_kg=0.0, load_band=None)   # mass: the profile's (bringup.yaml / rig)
LIMITS = dict(coulomb_pct=20.0, viscous_pct=25.0, kt_pct=5.0, backlash_deg=0.10, latency_mean_ms=0.5, latency_p95_ms=2.0)
RATIONALE = ("Coulomb ±20 % / viscous ±25 %: grease warm-up alone moves friction ≈ 10–20 %; K ±5 %: NdFeB magnets lose ≈ 0.1 %/K, "
             "a 40 K winding rise ≈ 4–5 %; backlash +0.1°: wear or a loosening stage, not warm-up; latency mean +0.5 ms, p95 +2 ms: "
             "the bus and the host must not get slower as the CPU heats (the Asimov throttling finding).")


def drift(a, b):
    """Start fit a, end fit b (sysid fits dict per joint) → per joint drift + verdicts."""
    out = {}
    for j in a:
        fa, fb = a[j], b.get(j)
        if fb is None:
            continue

        def pct(x, y):
            return None if x in (None, 0) or y is None else 100.0 * (y - x) / abs(x)
        d = dict(coulomb_pct=pct(fa["coulomb_nm"], fb["coulomb_nm"]), viscous_pct=pct(fa["viscous_nms"], fb["viscous_nms"]),
                 kt_pct=pct(fa["kt_scale"], fb["kt_scale"]),
                 backlash_deg=None if fa["backlash_rad"] is None or fb["backlash_rad"] is None else math.degrees(fb["backlash_rad"] - fa["backlash_rad"]),
                 latency_mean_ms=fb["latency_ms"]["mean"] - fa["latency_ms"]["mean"], latency_p95_ms=fb["latency_ms"]["p95"] - fa["latency_ms"]["p95"])
        ok = {}
        for k, lim in LIMITS.items():
            v = d[k]
            if v is None:
                ok[k] = None
            elif k in ("backlash_deg", "latency_mean_ms", "latency_p95_ms"):
                ok[k] = v <= lim                                   # growth only
            else:
                ok[k] = abs(v) <= lim
        out[j] = dict(drift={k: None if v is None else round(v, 3) for k, v in d.items()}, ok=ok)
    return out


def _twin_mass(bu, kg):
    """Put (kg) or take off (0) the soak weight in the twin's model (the gravity feed-forward): at the grasp site (arm) or
    on the rig's levers ({joint: kg})."""
    from sw.rig import capture, set_mass
    tw = bu.twin
    if not hasattr(tw, "_mass0"):
        tw._mass0 = capture(tw.m)
    set_mass(tw.m, kg, tw._mass0)


def _motion(bu, sup, seconds, cfg, log, samples):
    from sw import determinism
    from sw.bringup import D2R
    from sw.sysid import _loaded_centre
    centre, amp = {}, {}
    for j in bu.names:
        A = bu.jc(j)["jog"] * D2R
        centre[j], amp[j] = _loaded_centre(bu, sup, j, A), A
    bu._move(sup, centre, speed=bu.cfg["cal_speed"], settle=0.3)
    if cfg.get("mass_kg"):                                   # the soak weight, hung at the centre and known to the feed-forward
        from sw.sysid import _operator_mass
        _operator_mass(bu, sup, cfg["mass_kg"])
        _twin_mass(bu, cfg["mass_kg"])
    w = 2 * math.pi * cfg["freq_hz"]
    n = int(seconds / bu.dt)
    rated = np.array([bu.J[j]["rated_nm"] for j in bu.names])
    sq, cnt, stops = np.zeros(len(bu.names)), 0, []
    t_next = bu.now() + cfg["sample_s"]
    rej0 = sum(d.rejected for d in bu.drivers.values())
    for i in range(n):
        t = (i + 1) * bu.dt
        ramp = min(1.0, t / 2.0)
        for j in bu.names:
            a = amp[j] * ramp
            bu.hold_q[j] = centre[j] + a * math.sin(w * t)
            bu.hold_v[j] = a * w * math.cos(w * t)
            bu.hold_a[j] = -a * w * w * math.sin(w * t)
        v, fb = bu.cycle(sup, *bu._cmd_hold(bu.feedback()))
        if v.stop:
            stops.append(dict(t=round(t, 2), reasons=list(v.reasons)))
            break
        sq += fb.t ** 2
        cnt += 1
        if bu.now() >= t_next:
            t_next += cfg["sample_s"]
            s = determinism.snapshot(bu.buses)
            s["drive_temp_c"] = {j: bu.drivers[j].state.get("temp") for j in bu.names}
            s["t_s"] = round(t, 1)
            samples.append(s)
            log(f"  soak t={t:7.0f} s  CPU {s['cpu_temp_c']} °C  drives {s['drive_temp_c']}  throttle {s['throttled_flags'] or '—'}")
    for j in bu.names:
        bu.hold_v[j] = bu.hold_a[j] = 0.0
    rms = np.sqrt(sq / max(cnt, 1))
    return dict(seconds=round(cnt * bu.dt, 1), rms_torque_frac={j: round(float(rms[k] / rated[k]), 3) for k, j in enumerate(bu.names)},
                supervisor_stops=stops, rejected_frames=sum(d.rejected for d in bu.drivers.values()) - rej0)


def run(bu, seconds=None, cfg=None, log=print):
    """bu: a BringUp (virtual or real). → result dict with PASS / FAIL."""
    from sw import determinism, sysid
    from sw.bringup import Step
    cfg = {**SOAK, **(bu.cfg.get("soak") or {}), **(cfg or {})}
    seconds = seconds if seconds is not None else cfg["hours"] * 3600
    bu.run_all(until="calibrate")
    pre = {s: bu.steps[s].status for s in ("enumerate", "limits", "safety", "calibrate")}
    res = dict(definition=dict(cfg, limits=LIMITS, rationale=RATIONALE), pre=pre, virtual=bu.plant is not None,
               rig=bu.cfg.get("rig"), load_kg=bu.cfg.get("load_kg"))
    if any(v != "PASS" for v in pre.values()):
        res.update(status="FAIL", why=f"bring-up did not pass: {pre}")
        return res
    det0 = determinism.snapshot(bu.buses)
    s0 = Step("sysid_start")
    sysid.run(bu, s0)
    if not s0.data.get("fits"):
        res.update(status="FAIL", why="start fit failed: " + "; ".join(w for w, ok, d in s0.checks if not ok))
        return res
    n_per0 = len(bu.periods_ms)
    samples = []
    if bu.io.estop_active():
        bu.arm()
    sup = bu.supervisor(v_max=np.array([2.0 * bu.jc(j)["jog"] * math.pi / 180 * 2 * math.pi * cfg["freq_hz"] for j in bu.names]))
    bu.start(sup)
    bu.enable_all(sup)
    mot = _motion(bu, sup, seconds, cfg, log, samples)
    if cfg.get("mass_kg") and not mot["supervisor_stops"]:
        from sw.sysid import _operator_mass
        _twin_mass(bu, 0.0)
        _operator_mass(bu, sup, 0.0)
    bu.disable_all()
    s1 = Step("sysid_end")
    sysid.run(bu, s1)
    det1 = determinism.snapshot(bu.buses)
    hist = determinism.histogram(bu.periods_ms[n_per0:], bu.dt * 1000)
    D = drift(s0.data["fits"], s1.data.get("fits", {}))
    ver = determinism.verdict(det0, det1, hist, virtual=bu.plant is not None)
    fails = []
    for j, d in D.items():
        fails += [f"{j} {k} {d['drift'][k]:+.2f} (limit {LIMITS[k]})" for k, ok in d["ok"].items() if ok is False]
    if mot["supervisor_stops"]:
        fails.append(f"supervisor stop during the soak: {mot['supervisor_stops']}")
    if mot["rejected_frames"]:
        fails.append(f"{mot['rejected_frames']} implausible (corrupted) feedback frames during the soak")
    band = cfg.get("load_band")
    if band:                                                   # the soak ran at the intended load (Fable C1: it was 118 %)
        for j, f in mot["rms_torque_frac"].items():
            if not band[0] <= f <= band[1]:
                fails.append(f"{j}: RMS torque {f:.0%} of rated outside the soak load band {band[0]:.0%}–{band[1]:.0%} (wrong weights or lever?)")
    fails += [f"{w}: {dd}" for w, lvl, dd in ver if lvl == "❌"]
    res.update(status="FAIL" if fails else "PASS", fails=fails, motion=mot, drift=D,
               fits=dict(start={j: _short(f) for j, f in s0.data["fits"].items()}, end={j: _short(f) for j, f in s1.data.get("fits", {}).items()}),
               temps=dict(start={j: s0.data["rows"][j].get("temp") for j in s0.data["rows"]}),
               determinism=dict(verdict=ver, loop=hist, samples=samples, start=det0, end=det1),
               drive_temp_end={j: bu.drivers[j].state.get("temp") for j in bu.names})
    return res


def _short(f):
    return dict(coulomb_nm=f["coulomb_nm"], viscous_nms=f["viscous_nms"], kt_scale=f["kt_scale"], backlash_rad=f["backlash_rad"],
                latency_ms=dict(mean=f["latency_ms"]["mean"], p95=f["latency_ms"]["p95"]))


def write(res, name):
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"{name}.json").write_text(json.dumps(res, indent=1, default=str))
    d = res["definition"]
    L = [f"# Soak test {'(VIRTUAL rehearsal) ' if res['virtual'] else ''}— {res['status']} (generated by `python -m sw.soak`)", "",
         f"Machine: {'the ' + res['rig'] + ' lever rig (sw/config/rig_' + res['rig'] + '.yaml, docs/M1_RIG.md), session load ' + str(res.get('load_kg')) + ' kg of discs per lever' if res.get('rig') else 'the arm'}. "
         f"Definition: {d['hours']} h at ≈ {d['load_frac']:.0%} rated (0.25 Hz sine, soak weight hung for the motion {d['mass_kg']} kg"
         + (f", RMS torque band {d['load_band'][0]:.0%}–{d['load_band'][1]:.0%} of rated" if d.get("load_band") else "")
         + f"), sysid at the start and the end; limits {LIMITS}. {RATIONALE}", ""]
    if res.get("why"):
        L.append(f"**{res['why']}**")
    if res.get("motion"):
        m = res["motion"]
        L += [f"Motion: {m['seconds']} s, RMS torque {m['rms_torque_frac']} of rated, supervisor stops {len(m['supervisor_stops'])}, "
              f"implausible frames {m['rejected_frames']}; drive temperatures at the end {res.get('drive_temp_end')} °C.", "",
              "| Joint | F_c start → end | B | K | backlash ° | latency mean / p95 ms | drift | verdict |", "|---|---|---|---|---|---|---|---|"]
        for j, dd in res["drift"].items():
            a, b = res["fits"]["start"][j], res["fits"]["end"][j]
            bl = lambda x: "—" if x is None else f"{math.degrees(x):.3f}"
            L.append(f"| {j} | {a['coulomb_nm']:.4f} → {b['coulomb_nm']:.4f} | {a['viscous_nms']:.3f} → {b['viscous_nms']:.3f} | "
                     f"{a['kt_scale']} → {b['kt_scale']} | {bl(a['backlash_rad'])} → {bl(b['backlash_rad'])} | "
                     f"{a['latency_ms']['mean']:.1f}/{a['latency_ms']['p95']:.1f} → {b['latency_ms']['mean']:.1f}/{b['latency_ms']['p95']:.1f} | "
                     f"{dd['drift']} | {'✅' if all(v is not False for v in dd['ok'].values()) else '❌'} |")
        L += [""] + [f"- {lvl} {w}: {x}" for w, lvl, x in res["determinism"]["verdict"]]
    if res.get("fails"):
        L += ["", "**Fails:** " + "; ".join(res["fails"])]
    (OUT / f"{name}.md").write_text("\n".join(L) + "\n")
    return L


RIG_WEIGH_ERR_KG = 0.002     # virtual rig: mass on each weight bolt the twin does not know (the ±1 g kitchen-scale weighing, ×2)


def main(argv=None):
    from sw.bringup import RIG_CHOICES, ConsoleOperator, GpiodSafetyIO, build_virtual, load_cfg, rig_arg
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--bus", default="virtual")
    ap.add_argument("--estop", default=None)
    ap.add_argument("--joints", default=None)
    ap.add_argument("--hours", type=float, default=None)
    ap.add_argument("--seconds", type=float, default=None, help="override the duration (virtual default 60 s)")
    ap.add_argument("--mass", type=float, default=None)
    ap.add_argument("--fault", action="append", default=[])
    ap.add_argument("--profile", default="usb", help="virtual bus profile (sw/bus_sim.py)")
    ap.add_argument("--rig", default=None, choices=RIG_CHOICES, help="'step1' = the M1 Step-1 lever rig (docs/M1_RIG.md), 'arm' = "
                    "the arm. REQUIRED on real hardware; virtual default: arm")
    ap.add_argument("--load", default="soak", help="rig only: the session load (rig_<name>.yaml loads; the soak's discs by default)")
    a = ap.parse_args(argv)
    rig = rig_arg(ap, a)
    joints = a.joints.split(",") if a.joints else None
    extra = {} if a.mass is None else dict(mass_kg=a.mass)
    cfg = load_cfg()
    if rig:
        from sw.rig import apply
        cfg = apply(cfg, rig, a.load)
    if a.bus == "virtual":
        from sw.virtual_arm import SIM_GIM
        cfg["cal_speed"] = 0.5
        cfg["soak"] = dict(cfg.get("soak") or {}, sample_s=10.0)
        bu, arm = build_virtual(cfg=cfg, faults=a.fault, bus=a.profile, joints=joints, log=print, act={},
                                actuator_dir=Path(__file__).resolve().parent / "out" / "soak",
                                **(dict(gim_table=SIM_GIM, hand_extra_kg=RIG_WEIGH_ERR_KG) if rig else {}))
        secs = a.seconds if a.seconds is not None else (a.hours * 3600 if a.hours else 60.0)
        res = run(bu, secs, extra)
        name = "soak_virtual" + (f"_{rig}" if rig else "") + ("_" + "_".join(f.replace(":", "-") for f in a.fault) if a.fault else "")
    else:
        from sw.can_map import load
        from sw.drivers.canbus import CanBus
        from sw.bringup import BringUp
        cm = load(rig)
        kind, _, chans = a.bus.partition(":")
        chans = chans.split(",")
        buses, opened = {}, {}
        for i, bn in enumerate(cm["buses"]):
            ch = chans[i if len(chans) > 1 else 0]
            opened.setdefault(ch, CanBus(f"{kind}:{ch}"))
            buses[bn] = opened[ch]
        io = None
        if a.estop and a.estop.startswith("gpiod:"):
            _, chip, hb, aux = a.estop.split(":")
            io = GpiodSafetyIO(chip, hb, aux)
        bu = BringUp(buses, cm, cfg, io=io, operator=ConsoleOperator(), joints=joints, actuator_dir=Path(__file__).resolve().parent / "out" / "soak")
        if a.hours:
            extra["hours"] = a.hours
        res = run(bu, a.seconds, extra)
        name = f"soak_{time.strftime('%Y-%m-%d')}" + (f"_{rig}" if rig else "")
    print("\n".join(write(res, name)))
    raise SystemExit(0 if res["status"] == "PASS" else 1)


if __name__ == "__main__":
    main()
