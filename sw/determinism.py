"""Determinism checks for the bring-up report and the soak test (Asimov sim2real lessons 1–3: the robot should be one
deterministic plant, hour 20 like hour 1; PREEMPT_RT; a CPU that thermally throttles quietly degrades actuator response).
python -m sw.determinism        (prints a snapshot of this machine)

- **Loop period histogram** of the real control loop (wall clock, every cycle of the bring-up / soak), with the same
  percentiles as sw/loop_jitter.py; SW1 target: p99 |period − 2 ms| < 1 ms.
- **CPU temperature and throttling** on a Raspberry Pi: `vcgencmd get_throttled` (bits: 0 under-voltage, 1 ARM
  frequency capped, 2 throttled, 3 soft temperature limit — now; 16–19 the same, "has occurred since boot"), the
  thermal zone temperature and the ARM clock. Elsewhere the sysfs thermal zones / cpufreq if present; skipped if not.
- **Kernel real-time status:** PREEMPT_RT (`/sys/kernel/realtime`, `uname -v`), the preemption model, isolated CPUs,
  the cpufreq governor, this process's scheduling policy.
- **CAN controller health** per bus (sw/drivers/canbus.py `error_counters()`): state, TX/RX error counters, bus-off
  events, restarts, bus errors, dropped frames — before and after the run.
"""
from __future__ import annotations

import os
import platform
import re
import subprocess
from pathlib import Path

import numpy as np

THROTTLE_BITS = {0: "under-voltage now", 1: "ARM frequency capped now", 2: "throttled now", 3: "soft temperature limit now",
                 16: "under-voltage occurred", 17: "ARM frequency capping occurred", 18: "throttling occurred",
                 19: "soft temperature limit occurred"}
HIST_EDGES_MS = [0, 1.0, 1.5, 1.9, 2.1, 2.5, 3.0, 4.0, 6.0, 8.0, 12.0, 25.0, 1e9]


def _read(path):
    try:
        return Path(path).read_text().strip()
    except Exception:
        return None


def _run(cmd):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=2)
        return r.stdout.strip() if r.returncode == 0 else None
    except Exception:
        return None


def is_pi():
    m = _read("/proc/device-tree/model") or ""
    return "Raspberry Pi" in m, m.replace("\x00", "")


def throttled():
    """→ (raw int or None, [flag names]). vcgencmd first (Pi OS), then the firmware sysfs node (older kernels)."""
    out = _run(["vcgencmd", "get_throttled"])
    raw = None
    if out and "=" in out:
        try:
            raw = int(out.split("=")[1], 16)
        except ValueError:
            raw = None
    if raw is None:
        s = _read("/sys/devices/platform/soc/soc:firmware/get_throttled")
        if s:
            try:
                raw = int(s, 16)
            except ValueError:
                raw = None
    if raw is None:
        return None, []
    return raw, [n for b, n in THROTTLE_BITS.items() if raw & (1 << b)]


def cpu_temp_c():
    out = _run(["vcgencmd", "measure_temp"])
    if out:
        m = re.search(r"([\d.]+)", out)
        if m:
            return float(m.group(1))
    temps = []
    for z in sorted(Path("/sys/class/thermal").glob("thermal_zone*")):
        t = _read(z / "temp")
        if t and t.lstrip("-").isdigit():
            temps.append(int(t) / 1000.0)
    return max(temps) if temps else None


def cpu_freq():
    cur = _read("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq")
    mx = _read("/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq")
    gov = _read("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor")
    arm = _run(["vcgencmd", "measure_clock", "arm"])
    return dict(cur_mhz=None if not cur else int(cur) / 1000, max_mhz=None if not mx else int(mx) / 1000, governor=gov,
                vc_arm_hz=None if not arm else int(arm.split("=")[1]) if "=" in arm else None)


def rt_status():
    v = platform.version()
    rt_flag = _read("/sys/kernel/realtime")
    preempt = _read("/sys/kernel/debug/sched/preempt")              # root only; e.g. "none voluntary (full)"
    cmd = _read("/proc/cmdline") or ""
    iso = _read("/sys/devices/system/cpu/isolated")
    try:
        pol = os.sched_getscheduler(0)
        pol_name = {os.SCHED_OTHER: "SCHED_OTHER", getattr(os, "SCHED_FIFO", -1): "SCHED_FIFO", getattr(os, "SCHED_RR", -2): "SCHED_RR"}.get(pol, str(pol))
        prio = os.sched_getparam(0).sched_priority
    except Exception:
        pol_name, prio = None, None
    return dict(kernel=platform.release(), version=v, preempt_rt=(rt_flag == "1") or ("PREEMPT_RT" in v),
                preempt_model=("PREEMPT_RT" if "PREEMPT_RT" in v else "PREEMPT_DYNAMIC (not RT)" if "PREEMPT_DYNAMIC" in v else "PREEMPT (not RT)" if "PREEMPT" in v else "voluntary/none") if not preempt else preempt,
                isolated_cpus=iso or None, isolcpus_cmdline=bool(re.search(r"isolcpus=|nohz_full=", cmd)),
                sched_policy=pol_name, sched_priority=prio)


def snapshot(buses=None):
    pi, model = is_pi()
    raw, flags = throttled()
    can = {}
    for name, b in (buses or {}).items():
        try:
            can[name] = b.error_counters()
        except Exception as e:
            can[name] = dict(error=str(e))
    return dict(machine=model or platform.machine(), raspberry_pi=pi, cpu_temp_c=cpu_temp_c(), throttled_raw=raw,
                throttled_flags=flags, cpu_freq=cpu_freq(), rt=rt_status(), can=can, cpus=os.cpu_count(),
                loadavg=list(os.getloadavg()))


def histogram(periods_ms, period_ms=2.0):
    """Loop-period histogram + percentiles (the same as sw/loop_jitter.py) + jitter |period − nominal|."""
    e = np.asarray(periods_ms, float)
    if e.size == 0:
        return None
    counts, _ = np.histogram(e, bins=HIST_EDGES_MS)
    lab = [f"{a:g}–{b:g}" if b < 1e8 else f"> {a:g}" for a, b in zip(HIST_EDGES_MS[:-1], HIST_EDGES_MS[1:])]
    jit = np.abs(e - period_ms)
    pct = lambda x, p: round(float(np.percentile(x, p)), 3)
    return dict(n=int(e.size), period_ms=period_ms, bins_ms=lab, counts=counts.astype(int).tolist(),
                interval_ms=dict(p50=pct(e, 50), p99=pct(e, 99), p999=pct(e, 99.9), p9999=pct(e, 99.99), max=round(float(e.max()), 3)),
                jitter_ms=dict(p99=pct(jit, 99), p999=pct(jit, 99.9), max=round(float(jit.max()), 3)),
                over_ms={str(k): int(np.sum(e > k)) for k in (3, 4, 6, 8, 10, 15, 20, 25)})


def _can_delta(a, b):
    out = {}
    for name in set(a or {}) | set(b or {}):
        x, y = (a or {}).get(name) or {}, (b or {}).get(name) or {}
        if not y:
            out[name] = None
            continue
        d = {}
        for k, v in y.items():
            if isinstance(v, (int, float)) and isinstance(x.get(k), (int, float)):
                d[k] = v - x[k]
        out[name] = dict(state=y.get("state"), delta=d, after=y)
    return out


def verdict(start, end, hist, virtual=False):
    """→ list of (what, level, detail), level ✅ / ⚠️ / ❌. Warnings off the Pi are informational."""
    V = []
    pi = end.get("raspberry_pi")
    rt = end["rt"]
    V.append(("kernel real-time (PREEMPT_RT)", "✅" if rt["preempt_rt"] else ("⚠️" if pi else "ℹ️"),
              f"{rt['kernel']} ({rt['preempt_model']}); policy {rt['sched_policy']} prio {rt['sched_priority']}; isolated CPUs {rt['isolated_cpus'] or '—'}; "
              f"governor {end['cpu_freq'].get('governor')}" + ("" if pi else " — not the Pi: informational")))
    if end.get("throttled_raw") is None:
        V.append(("CPU throttling / under-voltage (vcgencmd get_throttled)", "ℹ️", "not available on this machine (skipped)"))
    else:
        new = sorted(set(end["throttled_flags"]) - set(start.get("throttled_flags") or []))
        bad = [f for f in end["throttled_flags"] if "occurred" in f or "now" in f]
        V.append(("CPU throttling / under-voltage (vcgencmd get_throttled)", "❌" if new or any("now" in f for f in bad) else ("⚠️" if bad else "✅"),
                  f"0x{end['throttled_raw']:x}: {', '.join(end['throttled_flags']) or 'none'}" + (f"; NEW during the run: {', '.join(new)}" if new else "")))
    t0, t1 = start.get("cpu_temp_c"), end.get("cpu_temp_c")
    if t1 is None:
        V.append(("CPU temperature", "ℹ️", "no sensor readable (skipped)"))
    else:
        V.append(("CPU temperature (Pi 5 firmware caps the clock from 80 °C)", "✅" if t1 < 70 else ("⚠️" if t1 < 80 else "❌"),
                  f"{t0} → {t1} °C" + ("" if pi else " (not the Pi)")))
    if hist:
        j = hist["jitter_ms"]
        V.append(("loop period (SW1: p99 |period − 2 ms| < 1 ms)", "ℹ️" if virtual else ("✅" if j["p99"] < 1.0 else "❌"),
                  f"p50 {hist['interval_ms']['p50']:.2f}, p99.9 {hist['interval_ms']['p999']:.2f}, max {hist['interval_ms']['max']:.2f} ms; jitter p99 {j['p99']:.2f} ms; "
                  f"> 8 ms: {hist['over_ms']['8']}" + (" — virtual run: compute time per cycle, not a real-time measurement" if virtual else "")))
    for name, d in _can_delta(start.get("can"), end.get("can")).items():
        if d is None:
            V.append((f"CAN {name} controller health", "ℹ️", "adapter does not report error counters"))
            continue
        a, dl = d["after"], d["delta"]
        bad = (dl.get("bus_off") or 0) > 0 or a.get("state") in ("BUS-OFF", "ERROR-PASSIVE")
        warn = (dl.get("bus_errors") or 0) > 0 or (dl.get("rx_dropped") or 0) > 0 or (dl.get("rx_over") or 0) > 0
        V.append((f"CAN {name} controller health", "❌" if bad else ("⚠️" if warn else "✅"),
                  f"state {a.get('state')}, TEC/REC {a.get('berr_tx')}/{a.get('berr_rx')}, during the run: bus-off +{dl.get('bus_off', 0)}, "
                  f"restarts +{dl.get('restarts', 0)}, bus errors +{dl.get('bus_errors', 0)}, RX dropped +{dl.get('rx_dropped', 0)}"))
    return V


def markdown(det):
    L = ["## Determinism (sw/determinism.py: Asimov lessons 1–3)", "", f"Machine: {det['end']['machine']} ({det['end']['cpus']} CPUs).", ""]
    L += [f"- {lvl} {w}: {d}" for w, lvl, d in det["verdict"]]
    h = det.get("loop")
    if h:
        L += ["", "| Loop period (ms) | " + " | ".join(h["bins_ms"]) + " |", "|---|" + "---|" * len(h["bins_ms"]),
              "| cycles | " + " | ".join(str(c) for c in h["counts"]) + " |"]
    return L


if __name__ == "__main__":
    import json
    print(json.dumps(snapshot(), indent=1))
