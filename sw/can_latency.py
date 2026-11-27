"""CAN interface latency / jitter / loss on the Pi (docs/sw/can_interface_note.md: gs_usb USB adapter vs MCP2518FD SPI HAT).
python -m sw.can_latency --pair can0,can2 [--seconds 60] [--hz 1000]   one-way: send on A, receive on B (same wire), kernel RX stamps
python -m sw.can_latency --drive can0 --id 5 [--seconds 60]           round trip: DM-J4310 status request → reply, at 500 Hz
python -m sw.can_latency --virtual                                     self-test of the code path (no timing meaning)

Wire both interfaces to the same bus (the Step-1 B2 cable: 2 × 120 Ω at the ends) and swap the roles to get both
directions. Reports the latency histogram and percentiles, lost frames, the error counters of both interfaces
(sw/drivers/canbus.py socketcan_counters) and the determinism snapshot (PREEMPT_RT, CPU temperature, throttling). Run it
idle and with the cameras / a CPU load running (`--stress 3`), on the stock and the PREEMPT_RT kernel.
"""
from __future__ import annotations

import argparse
import json
import os
import struct
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

OUT = Path(__file__).resolve().parent.parent / "docs" / "sw"


def _bus(spec):
    import can
    kind, _, ch = spec.partition(":") if ":" in spec else ("socketcan", "", spec)
    return can.Bus(interface=kind, channel=ch)


def pair(a, b, seconds=10.0, hz=1000.0, arb=0x123):
    """Send a counter + send time on A, receive on B: one-way latency (send call → B's kernel RX stamp) and loss."""
    import can
    ta, tb = _bus(a), _bus(b)
    lat, seen, n = [], set(), 0
    period = 1.0 / hz
    t_end = time.time() + seconds
    t_next = time.time()
    while time.time() < t_end:
        now = time.time()
        if now >= t_next:
            ta.send(can.Message(arbitration_id=arb, data=struct.pack("<II", n, 0),
                                is_extended_id=False))
            sent = now
            n += 1
            t_next += period
            m = tb.recv(0.02)
            while m is not None:
                if m.arbitration_id == arb:
                    k = struct.unpack("<I", bytes(m.data[:4]))[0]
                    if k not in seen:
                        seen.add(k)
                        if k == n - 1:
                            lat.append((m.timestamp - sent) * 1000.0)
                m = tb.recv(0.0)
        else:
            time.sleep(min(0.0002, t_next - now))
    ta.shutdown()
    tb.shutdown()
    return dict(mode="pair", a=a, b=b, sent=n, received=len(seen), lost=n - len(seen), latency_ms=_pct(lat))


def drive(spec, can_id=5, fb_id=0x15, seconds=10.0, hz=500.0):
    """DM-J4310 status request (0x7FF, CC) → its feedback frame on the master ID: round trip, at the control rate."""
    import can
    from sw.drivers import protocol as P
    from sw.drivers.mit import DM_J4310
    bus = _bus(spec)
    a, tpl = DM_J4310["REFRESH"]
    req = can.Message(arbitration_id=P.arb(a, can_id, fb_id), data=P.build(tpl, can_id), is_extended_id=False)
    rt, lost, n = [], 0, 0
    t_end = time.time() + seconds
    while time.time() < t_end:
        t0 = time.time()
        bus.send(req)
        n += 1
        m = bus.recv(0.02)
        while m is not None and m.arbitration_id != fb_id:
            m = bus.recv(0.02)
        if m is None:
            lost += 1
        else:
            rt.append((m.timestamp - t0) * 1000.0)
        time.sleep(max(0.0, 1.0 / hz - (time.time() - t0)))
    bus.shutdown()
    return dict(mode="drive", bus=spec, id=can_id, sent=n, lost=lost, round_trip_ms=_pct(rt))


def _pct(x):
    x = np.asarray(x, float)
    if not x.size:
        return None
    p = lambda q: round(float(np.percentile(x, q)), 4)
    from sw.determinism import histogram
    h = histogram(x, 0.0)
    return dict(n=int(x.size), mean=round(float(x.mean()), 4), p50=p(50), p99=p(99), p999=p(99.9), max=round(float(x.max()), 4),
                bins_ms=h["bins_ms"], counts=h["counts"])


def main(argv=None):
    from sw.determinism import snapshot
    from sw.drivers.canbus import socketcan_counters
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--pair", default=None)
    ap.add_argument("--drive", default=None)
    ap.add_argument("--id", type=lambda s: int(s, 0), default=5)
    ap.add_argument("--fb", type=lambda s: int(s, 0), default=0x15)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--hz", type=float, default=1000.0)
    ap.add_argument("--stress", type=int, default=0, help="busy-loop processes during the run (loaded case)")
    ap.add_argument("--virtual", action="store_true")
    ap.add_argument("--tag", default="")
    a = ap.parse_args(argv)
    hogs = [subprocess.Popen([sys.executable, "-c", "while True: pass"]) for _ in range(a.stress)]
    try:
        if a.virtual:
            p = f"lat{os.getpid()}"
            res = pair(f"virtual:{p}", f"virtual:{p}", min(a.seconds, 1.0), 200.0)
        elif a.pair:
            x, y = a.pair.split(",")
            c0 = {i: socketcan_counters(i) for i in (x, y)}
            res = pair(x, y, a.seconds, a.hz)
            res["counters"] = dict(before=c0, after={i: socketcan_counters(i) for i in (x, y)})
        elif a.drive:
            c0 = socketcan_counters(a.drive)
            res = drive(a.drive, a.id, a.fb, a.seconds)
            res["counters"] = dict(before=c0, after=socketcan_counters(a.drive))
        else:
            ap.error("--pair, --drive or --virtual")
    finally:
        for h in hogs:
            h.kill()
    res.update(stress=a.stress, platform=snapshot(), date=time.strftime("%Y-%m-%d %H:%M"))
    name = f"can_latency_{a.tag or res['mode']}"
    if not a.virtual:
        OUT.mkdir(parents=True, exist_ok=True)
        (OUT / f"{name}.json").write_text(json.dumps(res, indent=1, default=str))
    print(json.dumps({k: v for k, v in res.items() if k != "platform"}, indent=1, default=str))
    return res


if __name__ == "__main__":
    main()
