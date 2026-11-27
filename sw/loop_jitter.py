"""Heartbeat jitter of the real-time bring-up loop (Fable review M2: size the hardware watchdog on a measurement).
python -m sw.loop_jitter [--seconds 30]

Runs sw/bringup.py's own hardware path on this machine, in wall-clock time: `BringUp` with its `RealClock` (500 Hz,
time.sleep to the next deadline), python-can buses (interface "virtual"), the safety supervisor in every cycle, all five
CAN joints online (the SIM PLACEHOLDER GIM table + the DM-J4310 table) and enabled with zero gains, and a heartbeat
output that only timestamps its edges. A responder thread in the same process plays the five drives (sw/virtual_arm.py
`SimMotor`, arm held still) and answers every frame at once.

What it measures is the interval between successive heartbeat edges, i.e. what the SN74LVC1G123 monostable sees. It
includes Python, the supervisor, the protocol codec, python-can and the scheduler of this (non-real-time) Linux box. It
does NOT include the USB-CAN adapter's latency or the Pi 5 (slower cores); the responder thread competes for the GIL,
which the real kernel SocketCAN path does not (pessimistic). Re-run it on the Pi with the adapters before day 1
(same command; on the Pi add `--socketcan` once a real adapter loop-back is wired, TODO).

Writes docs/sw/loop_jitter.json + .md. rnd/electrical.py reads the JSON to set the watchdog timeout.
"""
from __future__ import annotations

import argparse
import json
import os
import platform
import threading
import time
from pathlib import Path

import numpy as np

OUT = Path(__file__).resolve().parent.parent / "docs" / "sw"


class TimingIO:
    """Heartbeat line that records its edges; contactor always closed (no e-stop circuit in this measurement)."""

    def __init__(self):
        self.edges = []
        self.record = False

    def heartbeat(self):
        if self.record:
            self.edges.append(time.perf_counter())

    def update(self):
        pass

    def estop_active(self):
        return False

    def arm(self):
        return True


class NullOperator:
    def arm_contactor(self):
        return True

    def say(self, text):
        pass

    def confirm(self, text):
        return True


def _responder(can_map, tables, prefix, stop):
    """The drives: answer every frame immediately (motor held at its zero)."""
    import can
    from sw.drivers import protocol as P
    from sw.virtual_arm import SimMotor
    rng = np.random.default_rng(0)
    J = {j["name"]: j for j in json.loads((Path(__file__).resolve().parent / "description" / "joints.json").read_text())}
    buses, motors = {}, []
    for bn, b in can_map["buses"].items():
        buses[bn] = can.Bus(interface="virtual", channel=f"{prefix}_{bn}")
        for dv in b["devices"]:
            tbl = tables.get(dv["kind"])
            if tbl is None or P.missing(tbl):
                continue
            mt = SimMotor(dv["name"], dv["kind"], dv["id"], dv["fb_id"], dv["ratio"], tbl, rng, J[dv["name"]]["rated_nm"])
            mt.bus = bn
            mt.power(True, 0.0)
            mt._last_p = 0.0
            motors.append(mt)
    send = {bn: (lambda a, d, _b=bus: _b.send(can.Message(arbitration_id=a, data=d, is_extended_id=False))) for bn, bus in buses.items()}
    ready = threading.Event()

    def run():
        ready.set()
        while not stop.is_set():
            idle = True
            for bn, bus in buses.items():
                msg = bus.recv(0.0)
                if msg is None:
                    continue
                idle = False
                for mt in motors:
                    if mt.bus == bn and mt.handle(msg.arbitration_id, bytes(msg.data), time.monotonic(), send[bn]):
                        mt.feedback(0.0, 0.0, send[bn])
                        break
            if idle:
                time.sleep(0.0001)
        for b in buses.values():
            b.shutdown()
    th = threading.Thread(target=run, daemon=True)
    th.start()
    ready.wait()
    return th


def measure(seconds=30.0, log=print):
    from sw.bringup import TABLES, BringUp, RealClock, load_cfg
    from sw.can_map import load
    from sw.drivers.canbus import CanBus
    from sw.virtual_arm import SIM_GIM
    cfg = load_cfg()
    cm = load()
    tables = dict(TABLES, gim6010=SIM_GIM)
    prefix = f"jit{os.getpid()}"
    stop = threading.Event()
    th = _responder(cm, tables, prefix, stop)
    buses = {bn: CanBus(f"virtual:{prefix}_{bn}") for bn in cm["buses"]}
    io = TimingIO()
    dt = 1.0 / cfg["control_hz"]
    clock = RealClock(dt)
    bu = BringUp(buses, cm, cfg, tables, io=io, operator=NullOperator(), clock=clock, reg_timeout=0.05, log=lambda *a: None)
    bu.step_enumerate()
    assert len(bu.names) == 5, (bu.names, bu.steps["enumerate"].checks)
    sup = bu.supervisor(limits=bu._raw_limits())
    bu.start(sup)
    bu.enable_all(sup)
    n = len(bu.names)
    z = np.zeros(n)
    fb = bu.feedback()
    q0 = fb.q.copy()
    for _ in range(250):                                    # warm-up, not recorded
        v, fb = bu.cycle(sup, q0, z, z, z)
    io.record = True
    ov0 = clock.overruns
    t_end = time.perf_counter() + seconds
    stops = 0
    while time.perf_counter() < t_end:
        v, fb = bu.cycle(sup, q0, z, z, z)
        stops += bool(v.stop)
    io.record = False
    stop.set()
    for d in bu.drivers.values():
        d.disable()
    th.join(1.0)
    for b in buses.values():
        b.close()
    e = np.diff(np.array(io.edges)) * 1000.0
    pct = lambda p: float(np.percentile(e, p))
    res = dict(date=time.strftime("%Y-%m-%d"), seconds=seconds, cycles=int(len(e)), period_ms=dt * 1000, joints=bu.names,
               supervisor_stops=stops, overruns=int(clock.overruns - ov0),
               interval_ms=dict(p50=round(pct(50), 3), p99=round(pct(99), 3), p999=round(pct(99.9), 3), p9999=round(pct(99.99), 3),
                                max=round(float(e.max()), 3), mean=round(float(e.mean()), 3)),
               over_ms={str(k): int(np.sum(e > k)) for k in (3, 4, 6, 8, 10, 15, 20, 25)},
               histogram=_hist(e, dt * 1000), platform=_platform(),
               machine=dict(cpus=os.cpu_count(), kernel=platform.release(), python=platform.python_version(),
                            loadavg_before=None, loadavg_after=list(os.getloadavg())),
               note="wall-clock run of BringUp.cycle (supervisor + codec + python-can virtual bus, responder thread in-process); "
                    "no USB-CAN adapter, not the Pi 5, not SCHED_FIFO")
    return res


def _hist(e, period_ms):
    from sw.determinism import histogram
    return histogram(e, period_ms)


def _platform():
    """RT kernel, CPU temperature and throttle flags at the end of the run (sw/determinism.py; skipped off the Pi)."""
    from sw.determinism import snapshot
    s = snapshot()
    return dict(machine=s["machine"], raspberry_pi=s["raspberry_pi"], preempt_rt=s["rt"]["preempt_rt"], preempt_model=s["rt"]["preempt_model"],
                cpu_temp_c=s["cpu_temp_c"], throttled=s["throttled_flags"], governor=s["cpu_freq"]["governor"])


def write(res):
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "loop_jitter.json").write_text(json.dumps(res, indent=1))
    iv = res["interval_ms"]
    L = ["# Bring-up loop heartbeat jitter (generated by `python -m sw.loop_jitter`)", "",
         f"{res['date']}: {res['cycles']} heartbeat intervals over {res['seconds']:.0f} s of `BringUp.cycle` at {1000 / res['period_ms']:.0f} Hz "
         f"(joints {', '.join(res['joints'])}, supervisor in the loop, python-can virtual bus, drives answered by an in-process thread) "
         f"on this machine ({res['machine']['cpus']} cores shared with other jobs, Linux {res['machine']['kernel']}, Python {res['machine']['python']}, "
         f"load average after {', '.join(f'{x:.2f}' for x in res['machine']['loadavg_after'])}).", "",
         "| Interval between heartbeat edges | ms |", "|---|---|"]
    L += [f"| {k} | {v:.2f} |" for k, v in iv.items()]
    L += ["", "| Intervals longer than | count |", "|---|---|"] + [f"| {k} ms | {v} |" for k, v in res["over_ms"].items()]
    h = res.get("histogram")
    if h:
        L += ["", "| Period histogram (ms) | " + " | ".join(h["bins_ms"]) + " |", "|---|" + "---|" * len(h["bins_ms"]),
              "| cycles | " + " | ".join(str(c) for c in h["counts"]) + " |", "",
              f"Jitter |period − {h['period_ms']:.0f} ms|: p99 {h['jitter_ms']['p99']:.2f}, p99.9 {h['jitter_ms']['p999']:.2f}, max {h['jitter_ms']['max']:.2f} ms (SW1: p99 < 1 ms)."]
    pf = res.get("platform")
    if pf:
        L += ["", f"Platform: {pf['machine']}, PREEMPT_RT {'yes' if pf['preempt_rt'] else 'no'} ({pf['preempt_model']}), CPU {pf['cpu_temp_c'] or '—'} °C, "
              f"throttle flags {', '.join(pf['throttled']) or 'none / not available'}, governor {pf['governor'] or '—'}."]
    if "loaded" in res:
        ld = res["loaded"]
        L += ["", f"**Loaded pass:** the same loop for {ld['seconds']:.0f} s ({ld['cycles']} intervals) with {ld['busy_processes']} busy-loop processes "
              f"competing for the cores: p50 {ld['interval_ms']['p50']:.2f}, p99.99 {ld['interval_ms']['p9999']:.2f}, max **{ld['interval_ms']['max']:.2f} ms**; "
              f"intervals over 8 ms: {ld['over_ms']['8']}, over 25 ms: {ld['over_ms']['25']}; overruns {ld['overruns']}."]
    L += ["", f"Supervisor stops during the run: {res['supervisor_stops']}; loop overruns (a deadline missed by a whole period): {res['overruns']}.", "",
          "**Use:** rnd/electrical.py sizes the A5 v2 watchdog monostable from this file (timeout ≥ the larger of 3 × the 99.99th "
          "percentile and 2 × the longest interval seen, rounded up to 5 ms, and the heartbeat-lost → no-torque chain ≤ the SW2 "
          "budget of 50 ms). **Caveat:** not the Pi 5 and no USB-CAN adapter. Re-run on the Pi before day 1 and raise the timeout "
          "if the longest interval there is longer; the safety step of sw/bringup.py also counts heartbeat gaps on the real bench."]
    (OUT / "loop_jitter.md").write_text("\n".join(L) + "\n")
    return L


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--stress", type=int, default=0, help="also run this many busy-loop processes during a second pass (loaded case)")
    a = ap.parse_args(argv)
    la = list(os.getloadavg())
    res = measure(a.seconds)
    res["machine"]["loadavg_before"] = la
    if a.stress:
        import subprocess
        import sys
        hogs = [subprocess.Popen([sys.executable, "-c", "while True: pass"]) for _ in range(a.stress)]
        try:
            r2 = measure(a.seconds / 2)
        finally:
            for h in hogs:
                h.kill()
        res["loaded"] = dict(busy_processes=a.stress, seconds=r2["seconds"], cycles=r2["cycles"], interval_ms=r2["interval_ms"],
                             over_ms=r2["over_ms"], overruns=r2["overruns"], supervisor_stops=r2["supervisor_stops"])
    print("\n".join(write(res)))


if __name__ == "__main__":
    main()
