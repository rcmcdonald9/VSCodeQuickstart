"""M1 Step 1a: the unloaded, current-limited motor tests that need no safety board (Fable review 2026-11-28, M1 and C1).
python -m sw.rig_1a --bus socketcan:can0 --joint yaw  limits | verify | impedance | timeout | stall   (bench, one phase)
python -m sw.rig_1a --bus virtual                                                                       (rehearse all)

Step 1a is the GIM6010-8 (and the DM-J4310) on the rig's motor plate with only the hub and a printed index disc on the
output (no lever, no weights: nothing to stop or guard), the 10 A bench supply at 24.0 V with its current limit at 1.0 A
as the power source, and its output button as the stop. It proves what everything after it relies on, before the
safety board and the lever are bought or fitted: `python -m sw.gim_discover` answers whether the SDC104 speaks an MIT /
impedance mode (listen, no-torque probe, zero-gain trace); once the GIM table in sw/drivers/gim.py is filled from the
manual and that report, this tool runs the rest, phase by phase (docs/M1_RIG.md §Step 1a has the procedure and the pass
criteria):

  limits     write the 1a torque limit (default 1.0 N·m: gentle at stall) and the 30 ms CAN timeout, save to flash, read back
  verify     after the operator switched the supply off for ≥ 10 s and on again: read both back (did flash keep them?)
  impedance  kp 1.0 N·m/rad, kd 0.05 N·m·s/rad: a +0.3 rad target ramp from where the disc sits, hold, and back; the output
             must follow within 0.05 rad and the reported torque must equal kp·e + kd·ė within 0.1 N·m (the MIT law)
  timeout    zero-gain frames for 0.5 s, then silence for 3× the timeout: the drive must then report itself disabled / timed
             out (its own CAN watchdog: the drive-side half of the stop chain)
  stall      the printed stall arm on the hub hooks a luggage scale anchored to the bench 100 mm from the axis (the output
             is blocked; no motion): the demand kp·(target − p) ramps 0 → 2 × the limit → 0 over 4 s (kp 10, target
             offset ≤ 0.2 rad: computed inside the drive, so only the drive's own limit can cap it); the reported torque
             must flatten at the limit (±10 %) and the scale's peak-hold must read limit / 0.100 m (±15 %; typed in)

The supply's 1 A limit caps power (24 W), NOT stall torque: at stall the phase current is ≈ √(24 W / R) ≈ 8 A for the
GIM (R ≈ 0.35 Ω, ESTIMATE) ≈ 3.9 N·m at the output. Hands off the disc whenever kp > 0 or τ_ff ≠ 0; the drive limit set
in `limits` is what bounds the stall test. `limits`, `verify` and `timeout` never command torque.
"""
from __future__ import annotations

import argparse
import datetime as _dt
import json
import math
import time
from pathlib import Path

import numpy as np

OUT = Path(__file__).resolve().parent.parent / "docs" / "sw"
DT = 0.002
LIMIT_1A_NM = 1.0
ARM_R = 0.100                    # m, stall arm hook radius (rnd/parts/rig.py stall_arm)
KP, KD, STEP_RAD = 5.0, 0.10, 0.15  # peak kp·e 0.75 N·m < the 1 N·m limit; friction dead-band F_c / kp ≈ 0.024 rad
KP_STALL = 10.0                     # stall: target offset up to 2 × limit / kp (0.2 rad at 1 N·m) against the blocked output
TOL = dict(track_rad=0.05, law_nm_p99=0.10, plateau_frac=0.10, scale_frac=0.15)


class RealBench:
    """The real CAN bus at 500 Hz; the operator at the console."""

    def __init__(self, bus):
        self.bus, self.next = bus, None

    def wait(self):
        now = time.monotonic()
        self.next = now if self.next is None or now > self.next + DT else self.next
        self.next += DT
        time.sleep(max(0.0, self.next - time.monotonic()))

    def ask(self, text):
        return input(f"\n>>> {text} ")

    def block(self, on):
        self.ask("FIT the stall arm, hook the luggage scale (anchored to the bench, cord square to the arm), hands clear, Enter"
                 if on else "REMOVE the stall arm and the scale, Enter")

    def power_cycle(self):
        self.ask("Switch the supply OFF, wait 10 s, switch it ON again (still 24.0 V / 1.0 A), then Enter")


def driver(bus, joint, tables, rig="step1"):
    from sw.drivers import protocol as P
    from sw.rig import can_map
    dv = next(d for b in can_map(rig)["buses"].values() for d in b["devices"] if d["name"] == joint)
    d = P.TableDriver(bus, joint, dv["id"], dv["fb_id"], dv["ratio"], tables[dv["kind"]])
    miss = d.missing()
    if miss:
        raise P.ProtocolTodo(f"{joint}: protocol table TODO {miss}: fill sw/drivers/gim.py from the manual and the "
                             "gim_discover report first (docs/M1_RIG.md 1a-4)")
    return d


def _stream(d, bench, n, fn):
    """n control periods: fn(i, state) → (q, v, kp, kd, tff) sent, then wait, then the newest feedback."""
    out = []
    for i in range(n):
        d.command(*fn(i, d.state))
        bench.wait()
        st = d.poll()
        if st.get("fresh"):
            out.append((i * DT, st["p"], st["v"], st["t"], st["status"]))
    return out


def _fresh(d, bench, n=10):
    for _ in range(n):
        d.refresh()
        bench.wait()
        if d.poll().get("fresh"):
            return d.state
    return None


def phase_limits(d, bench, limit_nm=LIMIT_1A_NM, timeout_s=0.030):
    t = d.table
    d.disable()
    bench.wait()
    raw = d.torque_limit_to_drive(limit_nm)
    tmo = int(round(timeout_s / t["CAN_TIMEOUT_UNIT_S"]))
    e1 = d.write_reg(t["TORQUE_LIMIT_REG"], raw)
    e2 = d.write_reg("can_timeout", tmo)
    d.save()
    bench.wait()
    rb, rt = d.read_reg(t["TORQUE_LIMIT_REG"]), d.read_reg("can_timeout")
    d.ranges_from_drive()
    ok = None not in (e1, e2, rb) and abs(rb - raw) <= 1e-4 * max(1.0, abs(raw)) and rt == tmo
    return dict(ok=ok, limit_nm=limit_nm, written=raw, readback=rb, timeout_counts=tmo, timeout_readback=rt)


def phase_verify(d, bench, limit_nm=LIMIT_1A_NM, timeout_s=0.030):
    t = d.table
    bench.power_cycle()
    st = _fresh(d, bench, 50)
    raw = d.torque_limit_to_drive(limit_nm)
    tmo = int(round(timeout_s / t["CAN_TIMEOUT_UNIT_S"]))
    rb, rt = d.read_reg(t["TORQUE_LIMIT_REG"]), d.read_reg("can_timeout")
    d.ranges_from_drive()
    ok = st is not None and rb is not None and abs(rb - raw) <= 1e-4 * max(1.0, abs(raw)) and rt == tmo
    return dict(ok=ok, answered_after_power_cycle=st is not None, readback=rb, timeout_readback=rt, expected=(raw, tmo))


def phase_impedance(d, bench):
    st = _fresh(d, bench)
    if st is None:
        return dict(ok=False, why="no feedback")
    q0 = st["p"]
    d.enable()
    bench.wait()
    T = (1.0, 1.0, 1.0, 1.0)                                        # ramp up, hold, ramp down, hold (s)
    n = int(sum(T) / DT)

    def target(i):
        t = i * DT
        if t < T[0]:
            return q0 + STEP_RAD * t / T[0], STEP_RAD / T[0]
        if t < T[0] + T[1]:
            return q0 + STEP_RAD, 0.0
        if t < T[0] + T[1] + T[2]:
            return q0 + STEP_RAD * (1 - (t - T[0] - T[1]) / T[2]), -STEP_RAD / T[2]
        return q0, 0.0
    tr = _stream(d, bench, n, lambda i, s: (target(i)[0], target(i)[1], KP, KD, 0.0))
    d.disable()
    bench.wait()
    A = np.array([(t, p, v, tau) for t, p, v, tau, _ in tr])
    tgt = np.array([target(int(round(t / DT)))[0] for t in A[:, 0]])
    vt = np.array([target(int(round(t / DT)))[1] for t in A[:, 0]])
    holds = [(T[0] + 0.5, T[0] + T[1]), (sum(T) - 0.5, sum(T))]
    err_hold = max(float(np.max(np.abs((A[:, 1] - tgt)[(A[:, 0] >= a) & (A[:, 0] < b)]))) for a, b in holds)
    law = KP * (tgt - A[:, 1]) + KD * (vt - A[:, 2])                 # the MIT law with the targets sent (one frame stale: lag)
    law_err = float(np.percentile(np.abs(A[5:, 3] - law[5:]), 99))
    bad = sorted({int(s) for *_, s in tr if s not in d.table["STATUS_OK"]})
    ok = err_hold < TOL["track_rad"] and law_err < TOL["law_nm_p99"] and not bad
    return dict(ok=ok, frames=len(tr), hold_error_rad=round(err_hold, 4), law_error_nm=round(law_err, 4), fault_status=bad,
                span_rad=round(float(A[:, 1].max() - A[:, 1].min()), 4))


def phase_timeout(d, bench, timeout_s=0.030):
    st = _fresh(d, bench)
    if st is None:
        return dict(ok=False, why="no feedback")
    d.enable()
    bench.wait()
    _stream(d, bench, int(0.5 / DT), lambda i, s: (s["p"], 0.0, 0.0, 0.0, 0.0))
    for _ in range(int(3 * timeout_s / DT)):                         # CAN silent: the drive's own watchdog must act
        bench.wait()
    st = _fresh(d, bench)
    t = d.table
    ok = st is not None and st["status"] in (t.get("STATUS_TIMEOUT"), t["STATUS_DISABLED"])
    d.disable()
    d.clear_error()
    bench.wait()
    return dict(ok=ok, status_after_silence=None if st is None else st["status"], silence_ms=round(3 * timeout_s * 1000, 1))


def phase_stall(d, bench, limit_nm=LIMIT_1A_NM):
    bench.block(True)
    st = _fresh(d, bench)
    if st is None:
        return dict(ok=False, why="no feedback")
    p0 = st["p"]
    d.enable()
    bench.wait()
    up, hold, down = 2.0, 1.0, 1.0
    n = int((up + hold + down) / DT)

    def dq(i):                                                       # the demand is kp·(target − p): computed IN the drive, so
        t = i * DT                                                   # neither the frame's T_MAX range nor the host can cap it
        peak = 2.0 * limit_nm / KP_STALL
        return peak * t / up if t < up else peak if t < up + hold else peak * max(0.0, 1 - (t - up - hold) / down)
    tr = _stream(d, bench, n, lambda i, s: (p0 + dq(i), 0.0, KP_STALL, 0.0, 0.0))
    scale = bench.ask(f"The scale's PEAK-HOLD reading? (kg; the arm is {ARM_R * 1000:.0f} mm, expect ≈ {limit_nm / ARM_R / 9.81:.2f} kg)")
    d.disable()
    bench.wait()
    bench.block(False)
    A = np.array([(t, tau) for t, p, v, tau, _ in tr])
    plateau = A[(A[:, 0] > up) & (A[:, 0] < up + hold), 1]
    pm = float(np.mean(np.abs(plateau))) if len(plateau) else float("nan")
    try:
        scale_nm = float(scale) * 9.81 * ARM_R
    except (TypeError, ValueError):
        scale_nm = float("nan")
    ok = abs(pm - limit_nm) <= TOL["plateau_frac"] * limit_nm and abs(scale_nm - limit_nm) <= TOL["scale_frac"] * limit_nm
    return dict(ok=ok, limit_nm=limit_nm, demanded_peak_nm=2 * limit_nm, reported_plateau_nm=round(pm, 4), scale_nm=round(scale_nm, 4),
                kt_check=round(scale_nm / pm, 4) if pm else None)


PHASES = {"limits": phase_limits, "verify": phase_verify, "impedance": phase_impedance, "timeout": phase_timeout, "stall": phase_stall}
ORDER = ["limits", "verify", "impedance", "timeout", "stall"]


# ------------------------------------------------------------------------------------------------ rehearsal
class SimBench:
    """One simulated drive (sw/virtual_arm.SimMotor: registers in RAM until saved, flash kept over a power cycle, torque
    clamped to its limit, its own CAN timeout) on a python-can virtual bus, driving the hub + index disc (or held by the
    stall arm), plus a simulated operator who power-cycles the supply and reads the scale."""

    def __init__(self, table, kind="gim6010", ratio=1.0, seed=0, J=0.0018, coulomb=0.12, faults=()):
        import can
        from sw.drivers.canbus import CanBus
        from sw.virtual_arm import SimMotor
        ch = f"rig1a{time.monotonic_ns()}"
        self.peer = can.Bus(interface="virtual", channel=ch)
        self.mt = SimMotor("yaw", kind, 0x03, 0x13, ratio, table, np.random.default_rng(seed), 5.0,
                           clamp="no_clamp" not in faults, timeout="no_timeout" not in faults)
        if "no_flash" in faults:                                      # a drive whose save does nothing
            self.mt.handle = _no_save(self.mt.handle, table)
        self.mt.power(True, 0.0)
        self.mt._last_p = self.mt.motor_pos(0.0)
        self.can, self.t, self.q, self.qd, self.J, self.fc = can, 0.0, 0.0, 0.0, J, coulomb
        self.blocked, self.tau_out, self.peak = False, 0.0, 0.0
        self.bus = CanBus(f"virtual:{ch}", clock=lambda: self.t)
        self.bus.pump = self._pump

    def _send(self, a, d):
        self.peer.send(self.can.Message(arbitration_id=a, data=d, is_extended_id=False))

    def _pump(self):
        while True:
            m = self.peer.recv(0.0)
            if m is None:
                return
            self.mt._last_p = self.mt.motor_pos(self.q)
            if self.mt.handle(m.arbitration_id, bytes(m.data), self.t, self._send):
                self.mt.feedback(self.q, self.qd, self._send)

    def wait(self):
        self._pump()
        for _ in range(4):                                           # 0.5 ms physics steps
            h = DT / 4
            tau = self.mt.torque(self.q, self.qd, self.t)
            self.tau_out = tau
            if self.blocked:
                self.qd = 0.0
                self.peak = max(self.peak, abs(tau))
            else:
                acc = (tau - self.fc * math.tanh(self.qd / 0.02)) / self.J
                self.qd += acc * h
                self.q += self.qd * h
            self.t += h
        self.mt._last_p = self.mt.motor_pos(self.q)

    def ask(self, text):
        return f"{self.peak / ARM_R / 9.81:.3f}"                       # the scale's peak hold: the true output torque

    def block(self, on):
        self.blocked, self.peak = on, 0.0

    def power_cycle(self):
        self.mt.power(False, self.t)
        self.mt.power(True, self.t)

    def close(self):
        self.bus.close()
        self.peer.shutdown()


def _no_save(handle, table):
    from sw.drivers import protocol as P

    def h(arb_id, data, now, send):
        a, tpl = table["PARAM_SAVE"]
        f = P.parse(tpl, data) if arb_id == P.arb(a, 0x03, 0x13) else None
        if f is not None and f.get("id") == 0x03:
            return True                                               # swallowed: RAM only
        return handle(arb_id, data, now, send)
    return h


def rehearse(log=print, table=None, faults=()):
    """Every phase against a simulated drive (SIM PLACEHOLDER table); faults: no_clamp, no_timeout, no_flash."""
    from sw.virtual_arm import SIM_GIM
    sb = SimBench(table or SIM_GIM, faults=faults)
    d = driver(sb.bus, "yaw", {"gim6010": table or SIM_GIM, "dm_j4310": table or SIM_GIM})
    res = {}
    for ph in ORDER:
        res[ph] = PHASES[ph](d, sb)
        log(f"[{'PASS' if res[ph]['ok'] else 'FAIL':4s}] {ph}: {res[ph]}")
    sb.close()
    return res


def write(res, tag):
    OUT.mkdir(parents=True, exist_ok=True)
    date = _dt.date.today().isoformat()
    (OUT / f"rig_1a_{date}{tag}.json").write_text(json.dumps(res, indent=1, default=str))
    L = [f"# M1 Step 1a motor tests {date}{tag} (generated by `python -m sw.rig_1a`)", "",
         "Unloaded motor on the rig's plate, hub + index disc only, supply 24.0 V / 1.0 A, no safety board (docs/M1_RIG.md §Step 1a).", "",
         "| Phase | Result | Data |", "|---|---|---|"] + [f"| {k} | {'✅' if v['ok'] else '❌'} | {v} |" for k, v in res.items()]
    (OUT / f"rig_1a_{date}{tag}.md").write_text("\n".join(L) + "\n")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--bus", default="virtual")
    ap.add_argument("--joint", default="yaw", choices=["yaw", "wpitch"])
    ap.add_argument("--limit-nm", type=float, default=LIMIT_1A_NM)
    ap.add_argument("phases", nargs="*", default=[], help=f"one or more of {ORDER} (virtual: all)")
    a = ap.parse_args(argv)
    if a.bus == "virtual":
        res = rehearse()
        write(res, "_virtual")
        raise SystemExit(0 if all(v["ok"] for v in res.values()) else 1)
    from sw.bringup import TABLES
    from sw.drivers.canbus import CanBus
    bad = [p for p in a.phases if p not in PHASES]
    if not a.phases or bad:
        ap.error(f"name the phases to run on the bench, from {ORDER}")
    print("STEP 1a: motor on its plate, hub + index disc only (no lever), supply 24.0 V / 1.0 A, supply OFF button in reach, "
          "hands off the output whenever it is enabled.")
    bus = CanBus(a.bus)
    bench = RealBench(bus)
    d = driver(bus, a.joint, TABLES)
    res = {}
    for ph in a.phases:
        kw = {"limit_nm": a.limit_nm} if ph in ("limits", "verify", "stall") else {}
        res[ph] = PHASES[ph](d, bench, **kw)
        print(f"[{'PASS' if res[ph]['ok'] else 'FAIL':4s}] {ph}: {res[ph]}")
    bus.close()
    write(res, f"_{a.joint}")
    raise SystemExit(0 if all(v["ok"] for v in res.values()) else 1)


if __name__ == "__main__":
    main()
