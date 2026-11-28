"""Bring-up step 1 for a SteadyWin GIM: CAN protocol discovery on ONE motor (Fable review M4).
python -m sw.gim_discover --bus socketcan:can0 --id 1 [--enable]        (the Step-1 GIM6010-8, alone on the bus)
python -m sw.gim_discover --bus virtual                                  (rehearsal against a simulated MIT-style drive)

The safety concept (sw/safety.py: total-torque bound kp·e + kd·ė + τ_ff, torque feedback for the collision observer, the
drive's CAN timeout, a torque limit in flash) assumes an MIT / impedance mode on every GIM driver. Nothing public confirms
that for the SDC104 (GIM6010-8), the SDC303 (GIM8108-36) or the GIM8108-8's driver (docs/sw/gim_protocol_notes.md).
So before any other GIM is bought, this tool finds out what the one Step-1 motor speaks, in three phases, and writes
docs/sw/gim_discovery_<date>.md + .json. It never commands torque unless `--enable` is given and the operator confirms,
and then only a zero-gain MIT frame (kp = kd = τ_ff = 0: no torque even if the layout is right).

1. **Listen** (2 s, no frames sent): does the drive broadcast anything on its own (status/heartbeat IDs)?
2. **Probe, no torque:** the documented Ben Katz / MIT "exit motor mode" frame (FF×7 FD), which also answers with a
   feedback frame on MIT-family firmware, sent to the configured node ID; then the same to IDs 1–8 if nothing answered.
   Every reply is logged raw and decoded with the MIT feedback layout under the candidate ranges (P_MAX 12.5 / 95.5,
   V_MAX 30 / 45 / 65, T_MAX 10 / 18 / 54), and the decode is checked for plausibility (finite, |p| < P_MAX, the ID nibble).
3. **Optional, `--enable` (motor in its fixture, PSU limited to 1 A):** enter motor mode (FF×7 FC), 200 ms of zero-gain
   MIT frames while the operator turns the output by hand (the decoded position must follow), exit motor mode (FF×7 FD).

What it cannot discover (needs the manual or the vendor tool, and stays TODO in sw/drivers/gim.py until then): register
numbers (IDs, CAN timeout, current/torque limit, flash save), status codes, whether the limit survives a power cycle.
Exit criteria for Step 1 of the M1 kit: docs/M1_READINESS.md.
"""
from __future__ import annotations

import argparse
import datetime as _dt
import json
import math
import time
from pathlib import Path

from sw.drivers.mit import DISABLE, ENABLE, Ranges, pack_cmd, unpack_feedback

OUT = Path(__file__).resolve().parent.parent / "docs" / "sw"
CANDIDATE_RANGES = [Ranges(12.5, 30.0, 10.0), Ranges(12.5, 65.0, 18.0), Ranges(95.5, 45.0, 18.0), Ranges(12.5, 45.0, 54.0)]


def _decode(data):
    """MIT feedback decodes under each candidate range; plausibility = finite and inside the range."""
    if len(data) != 8:
        return []
    out = []
    for r in CANDIDATE_RANGES:
        f = unpack_feedback(data, r)
        ok = all(math.isfinite(f[k]) for k in ("p", "v", "t")) and abs(f["p"]) < r.p_max * 0.999
        out.append(dict(ranges=(r.p_max, r.v_max, r.t_max), id=f["id"], err=f["err"], p=round(f["p"], 4), v=round(f["v"], 3),
                        t=round(f["t"], 3), temp=(f["t_mos"], f["t_rotor"]), plausible=bool(ok)))
    return out


def _collect(bus, seconds):
    """All frames for `seconds` (raw: arbitration ID, data hex)."""
    t_end = time.monotonic() + seconds
    seen = []
    while time.monotonic() < t_end:
        bus.drain()
        for aid, q in list(bus.inbox.items()):
            while q:
                st, d = q.popleft()
                seen.append((aid, d))
        time.sleep(0.002)
    return seen


def discover(bus, node_id=1, enable=False, confirm=lambda text: False, turn=None, log=print):
    rep = dict(date=_dt.date.today().isoformat(), node_id=node_id, phases={})
    bus.clear()
    idle = _collect(bus, 2.0)
    rep["phases"]["listen"] = dict(frames=len(idle), ids=sorted({a for a, _ in idle}),
                                  sample=[(hex(a), d.hex(" ")) for a, d in idle[:8]])
    log(f"listen: {len(idle)} frames, IDs {[hex(a) for a in rep['phases']['listen']['ids']]}")
    probes = []
    for nid in [node_id] + [i for i in range(1, 9) if i != node_id]:
        bus.send(nid, DISABLE)                                   # exit motor mode: no torque on any MIT-family drive
        rx = _collect(bus, 0.05)
        probes.append(dict(sent_to=nid, frame=DISABLE.hex(" "), replies=[dict(arb=hex(a), data=d.hex(" "), mit=_decode(d)) for a, d in rx]))
        if rx and nid == node_id:
            break
    rep["phases"]["probe"] = probes
    answered = [p for p in probes if p["replies"]]
    mit_ok = [p for p in answered if any(any(c["plausible"] for c in r["mit"]) for r in p["replies"])]
    log(f"probe: {len(answered)} ID(s) answered the exit-motor-mode frame; MIT-plausible replies from {[p['sent_to'] for p in mit_ok]}")
    if enable and mit_ok and confirm("Motor in its fixture, PSU limited to 1 A, hands clear except to turn the output: enter motor mode with ZERO gains?"):
        nid = mit_ok[0]["sent_to"]
        r = CANDIDATE_RANGES[0]
        bus.send(nid, ENABLE)
        trace = []
        t_end = time.monotonic() + 0.2
        while time.monotonic() < t_end:
            bus.send(nid, pack_cmd(0.0, 0.0, 0.0, 0.0, 0.0, r))     # kp = kd = τ_ff = 0: no torque
            for a, d in _collect(bus, 0.004):
                f = unpack_feedback(d, r)
                trace.append((round(f["p"], 4), round(f["v"], 3), round(f["t"], 3), f["err"]))
            if turn is not None:
                turn()
        bus.send(nid, DISABLE)
        _collect(bus, 0.02)
        ps = [x[0] for x in trace]
        rep["phases"]["enable"] = dict(node=nid, frames=len(trace), p_span=round(max(ps) - min(ps), 4) if ps else None,
                                      t_max_seen=max((abs(x[2]) for x in trace), default=None), err_codes=sorted({x[3] for x in trace}))
        log(f"enable (zero gains): {len(trace)} feedback frames, position span {rep['phases']['enable']['p_span']} rad while turned by hand")
    v = ("MIT-family frames answered (exit-motor-mode → MIT feedback layout plausible): fill CONTROL/FEEDBACK/ENABLE/DISABLE/CODEC "
         "in sw/drivers/gim.py, then the ranges from the vendor tool" if mit_ok else
         "no MIT-style answer: the drive may run SteadyWin's own protocol (forum reports for GIM8115-9) or need MIT mode selected "
         "in the vendor tool first; do not buy more GIMs until this is resolved" if answered else
         "no answer at all: check bit rate (1 Mbit/s), termination, node ID with the vendor tool over USB-C, then re-run")
    rep["verdict"] = v
    rep["mit_plausible"] = bool(mit_ok)
    return rep


def write(rep, tag=""):
    OUT.mkdir(parents=True, exist_ok=True)
    jp = OUT / f"gim_discovery_{rep['date']}{tag}.json"
    jp.write_text(json.dumps(rep, indent=1, default=str))
    L = [f"# GIM protocol discovery {rep['date']}{tag} (generated by `python -m sw.gim_discover`)", "",
         f"Node ID tried first: {rep['node_id']}. **Verdict:** {rep['verdict']}.", "",
         f"- Listen (2 s, nothing sent): {rep['phases']['listen']['frames']} frames on {rep['phases']['listen']['ids']}",
         ] + [f"- Probe → ID {p['sent_to']}: " + ("; ".join(f"{r['arb']} [{r['data']}] MIT plausible under "
              f"{[c['ranges'] for c in r['mit'] if c['plausible']]}" for r in p["replies"]) or "no reply") for p in rep["phases"]["probe"]]
    if "enable" in rep["phases"]:
        e = rep["phases"]["enable"]
        L.append(f"- Enable with zero gains on ID {e['node']}: {e['frames']} frames, position span {e['p_span']} rad, |τ| ≤ {e['t_max_seen']}, status codes {e['err_codes']}")
    L += ["", "Still needs the manual / vendor tool: register numbers (IDs, CAN timeout, current or torque limit, flash save), status "
          "codes, P/V/T ranges as configured, whether the limit survives a power cycle. See docs/sw/gim_protocol_notes.md."]
    (OUT / f"gim_discovery_{rep['date']}{tag}.md").write_text("\n".join(L) + "\n")
    return jp


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--bus", default="virtual")
    ap.add_argument("--id", type=lambda x: int(x, 0), default=1, help="node ID (decimal or 0x..)")
    ap.add_argument("--enable", action="store_true", help="phase 3: zero-gain motor mode (asks for confirmation)")
    a = ap.parse_args(argv)
    from sw.drivers.canbus import CanBus
    if a.bus == "virtual":
        rep = rehearse(enable=a.enable)
        tag = "_virtual"
    else:
        bus = CanBus(a.bus)
        print("Step 1a: the GIM alone on the bus on its rig plate, hub + index disc only, supply 24.0 V / 1.0 A, its OFF button "
              "in reach (docs/M1_RIG.md).")
        rep = discover(bus, a.id, a.enable, confirm=lambda t: input(t + " [type yes] ").strip() == "yes")
        bus.close()
        tag = ""
    print(f"report: {write(rep, tag)}")


def rehearse(enable=True, log=print):
    """Against a simulated drive that speaks the MIT family (the SIM PLACEHOLDER table): the tool's own test."""
    import numpy as np
    import can
    from sw.drivers.canbus import CanBus
    from sw.virtual_arm import SIM_GIM, SimMotor
    ch = f"gimdisc{time.monotonic_ns()}"
    peer = can.Bus(interface="virtual", channel=ch)
    mt = SimMotor("yaw", "gim6010", 1, 0x11, 1.0, SIM_GIM, np.random.default_rng(0), 10.0)
    mt.power(True, 0.0)
    mt._last_p = 0.0
    send = lambda a, d: peer.send(can.Message(arbitration_id=a, data=d, is_extended_id=False))
    q = [0.0]

    def pump():
        while True:
            m = peer.recv(0.0)
            if m is None:
                return
            if mt.handle(m.arbitration_id, bytes(m.data), time.monotonic(), send):
                mt.feedback(q[0], 0.0, send)
    bus = CanBus(f"virtual:{ch}")
    bus.pump = pump

    def turn():
        q[0] += 0.01
    rep = discover(bus, 1, enable, confirm=lambda t: True, turn=turn, log=log)
    bus.close()
    peer.shutdown()
    return rep


if __name__ == "__main__":
    main()
