"""CAN bus impairments for the virtual bench (sw/virtual_arm.py): latency, jitter, drops, duplicates, corruption, CAN
error counters and bus-off.  Lesson from the Asimov sim2real write-up: on-robot comms are noisy, lossy and late, and
their processor-in-the-loop sim injected CAN delays up to ≈ 9 ms. This module lets the bring-up, the safety supervisor
and the collision observer run against that instead of an ideal wire.

Each bus direction (host → drive "tx", drive → host "rx") passes every frame through `Impairment.pass_frame`, which
returns zero (dropped / CRC-rejected / bus-off), one or two (duplicated) copies with a release time. Releases are kept
in FIFO order per direction (a USB adapter or a driver queue delays frames, it does not reorder them), so a long delay
followed by a short one bunches frames: two arrive in one control cycle and the next cycle gets none. That bunching,
not the mean delay, is what stresses a 500 Hz loop.

Latency model (ms, per frame): fixed + jitter + tail, clipped to [0, max]:
- `fixed_ms`: constant part (USB frame scheduling, driver, adapter firmware);
- `jitter`: ("uniform", lo, hi) | ("exp", mean) | None;
- `tail`: (probability, max_ms): with that probability an extra uniform(0, max_ms) delay (USB hub/host-controller
  stalls, a busy kernel thread, a CPU frequency step);
- `max_ms` clip (9 ms: the worst case injected by the Asimov PIL sim).

Faults on the wire:
- `p_drop`: frame lost without trace (adapter/driver queue overflow; the classic MCP2515 two-buffer overrun);
- `p_dup`: frame delivered twice (a retransmission after a missed ACK; the copy 0.1–0.5 ms later);
- `p_crc`: frame destroyed on the wire and rejected by CRC: lost, and the error counters move (TEC +8 / REC +1, −1 per
  good frame: ISO 11898-1 fault confinement); > 127 error-passive, > 255 bus-off;
- `p_corrupt`: **app-level corruption**: one random bit flipped in the data *after* the CRC check (what a buggy native
  CAN controller driver or adapter firmware does; a correct CAN stack never delivers this);
- `bus_off_at` / `bus_off_ms`: forced bus-off at a time (s) for a duration, then auto-restart (`restart-ms`).

PROFILES are named presets; `make(profile_or_dict)` builds one. "ideal" is a no-op (the existing tests run on it).
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np

PROFILES = {
    # no impairment: identical to the plain python-can virtual bus
    "ideal": {},
    # a healthy gs_usb / candleLight adapter on a lightly loaded Pi: ≈ 0.1–0.6 ms, rare multi-ms stalls (ESTIMATE until
    # measured with python -m sw.can_latency on the Pi; docs/sw/can_interface_note.md)
    "usb": dict(fixed_ms=0.15, jitter=("exp", 0.25), tail=(0.002, 4.0), max_ms=9.0, p_drop=1e-5, p_dup=1e-5, p_crc=1e-5),
    # the Asimov processor-in-the-loop case: any delay 0–9 ms, every frame (the worst case we design for)
    "pil9": dict(fixed_ms=0.0, jitter=("uniform", 0.0, 9.0), max_ms=9.0, p_drop=1e-3, p_dup=1e-3, p_crc=1e-3),
    # a marginal bus (bad termination / long stub / EMI): 1 % lost, 0.5 % duplicated, CRC errors
    "lossy": dict(fixed_ms=0.2, jitter=("exp", 0.5), tail=(0.01, 6.0), max_ms=9.0, p_drop=0.01, p_dup=0.005, p_crc=0.005),
    # a buggy controller driver / firmware that delivers corrupted payloads (Asimov's native-CAN problem)
    "corrupt": dict(fixed_ms=0.2, jitter=("exp", 0.3), max_ms=9.0, p_corrupt=0.002),
}


@dataclass
class Counters:
    sent: int = 0
    delivered: int = 0
    dropped: int = 0
    duplicated: int = 0
    corrupted: int = 0
    crc_errors: int = 0
    bus_off_drops: int = 0
    delays_ms: list = field(default_factory=list)


class Impairment:
    def __init__(self, spec=None, rng=None, name=""):
        spec = dict(PROFILES[spec] if isinstance(spec, str) else (spec or {}))
        self.name = name
        self.spec = spec
        self.ideal = not any(v for k, v in spec.items() if k not in ("max_ms",))
        self.rng = rng if rng is not None else np.random.default_rng(12345)
        self.fixed = float(spec.get("fixed_ms", 0.0))
        self.jitter = spec.get("jitter")
        self.tail = spec.get("tail")
        self.max_ms = float(spec.get("max_ms", 9.0))
        self.p_drop, self.p_dup = float(spec.get("p_drop", 0.0)), float(spec.get("p_dup", 0.0))
        self.p_crc, self.p_corrupt = float(spec.get("p_crc", 0.0)), float(spec.get("p_corrupt", 0.0))
        self.bus_off_at, self.bus_off_ms = spec.get("bus_off_at"), float(spec.get("bus_off_ms", 100.0))
        self.last = {"tx": 0.0, "rx": 0.0}
        self.c = {"tx": Counters(), "rx": Counters()}
        self.tec = self.rec = 0
        self.bus_off_events = 0
        self.restarts = 0
        self.state = "ERROR-ACTIVE"
        self._off_until = None

    # ---- latency
    def sample_ms(self):
        d = self.fixed
        j = self.jitter
        if j:
            if j[0] == "uniform":
                d += float(self.rng.uniform(j[1], j[2]))
            elif j[0] == "exp":
                d += float(self.rng.exponential(j[1]))
        if self.tail and self.rng.random() < self.tail[0]:
            d += float(self.rng.uniform(0.0, self.tail[1]))
        return min(max(d, 0.0), self.max_ms)

    def _err_state(self):
        e = max(self.tec, self.rec)
        self.state = "BUS-OFF" if self.tec > 255 else ("ERROR-PASSIVE" if e > 127 else ("ERROR-WARNING" if e > 95 else "ERROR-ACTIVE"))

    def _bus_off(self, now):
        if self._off_until is not None:
            if now < self._off_until:
                return True
            self._off_until = None                 # auto-restart (SocketCAN restart-ms)
            self.tec = self.rec = 0
            self.restarts += 1
            self._err_state()
        if self.bus_off_at is not None and now >= self.bus_off_at:
            self.bus_off_at = None
            self.tec = 256
            self._err_state()
            self.bus_off_events += 1
            self._off_until = now + self.bus_off_ms / 1000.0
            return True
        return False

    def pass_frame(self, direction, now, data):
        """→ list of (release_time_s, data). Ideal: [(now, data)] with no random draws."""
        c = self.c[direction]
        c.sent += 1
        if self.ideal:
            c.delivered += 1
            return [(now, data)]
        if self._bus_off(now):
            c.bus_off_drops += 1
            return []
        r = self.rng.random(4)
        if r[0] < self.p_crc:                        # destroyed on the wire, CRC rejects it, error counters move
            c.crc_errors += 1
            if direction == "tx":
                self.tec += 8
            else:
                self.rec += 1
            self._err_state()
            if self.tec > 255:
                self.bus_off_events += 1
                self._off_until = now + self.bus_off_ms / 1000.0
            return []
        self.tec, self.rec = max(self.tec - 1, 0), max(self.rec - 1, 0)
        self._err_state()
        if r[1] < self.p_drop:
            c.dropped += 1
            return []
        if r[2] < self.p_corrupt:
            b = bytearray(data)
            bit = int(self.rng.integers(0, 8 * len(b)))
            b[bit // 8] ^= 1 << (bit % 8)
            data = bytes(b)
            c.corrupted += 1
        d = self.sample_ms()
        t = max(now + d / 1000.0, self.last[direction])            # FIFO: delayed, never reordered
        self.last[direction] = t
        c.delays_ms.append((t - now) * 1000.0)
        out = [(t, data)]
        c.delivered += 1
        if r[3] < self.p_dup:
            t2 = t + float(self.rng.uniform(1e-4, 5e-4))
            self.last[direction] = t2
            out.append((t2, data))
            c.duplicated += 1
        return out

    # ---- what a SocketCAN adapter would report (ip -details -statistics link show canX)
    def error_counters(self):
        tx, rx = self.c["tx"], self.c["rx"]
        return dict(state=self.state, berr_tx=self.tec, berr_rx=self.rec, bus_off=self.bus_off_events, restarts=self.restarts,
                    bus_errors=tx.crc_errors + rx.crc_errors, rx_dropped=rx.dropped, tx_dropped=tx.dropped)

    def stats(self):
        out = {}
        for k, c in self.c.items():
            d = np.array(c.delays_ms) if c.delays_ms else np.zeros(1)
            out[k] = dict(sent=c.sent, delivered=c.delivered, dropped=c.dropped, duplicated=c.duplicated, corrupted=c.corrupted,
                          crc_errors=c.crc_errors, bus_off_drops=c.bus_off_drops,
                          delay_ms=dict(mean=round(float(d.mean()), 3), p50=round(float(np.percentile(d, 50)), 3),
                                        p99=round(float(np.percentile(d, 99)), 3), max=round(float(d.max()), 3)))
        out["errors"] = self.error_counters()
        return out


def make(spec, rng=None, name=""):
    return spec if isinstance(spec, Impairment) else Impairment(spec, rng, name)


class DelayLine:
    """FIFO of (release, payload) for one bus direction."""

    def __init__(self):
        self.q = deque()

    def push(self, items, meta=None):
        for t, d in items:
            self.q.append((t, d, meta))

    def due(self, now):
        out = []
        while self.q and self.q[0][0] <= now + 1e-12:
            out.append(self.q.popleft())
        return out

    def __len__(self):
        return len(self.q)
