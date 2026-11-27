"""CAN bus adapter over python-can (S1). One object per physical bus (B1, B2 in sw/config/can_map.yaml).

The drivers only call `send(arb_id, data)` and `recv_for(arb_id, match=None, timeout=0)`; this class turns that into
python-can, so the same code runs on
- `virtual:<name>`      python-can's in-process virtual bus (sw/virtual_arm.py answers as the motors),
- `socketcan:<if>`      Linux SocketCAN (the candleLight / gs_usb adapters show up as can0, can1; set the bit rate with
                        `sudo ip link set can0 up type can bitrate 1000000`),
- `slcan:<tty>[@bitrate]` serial-line CAN adapters (CANable with slcan firmware),
- `gs_usb:<index>[@bitrate]` gs_usb through libusb (Windows / macOS).
Received frames are stamped with the caller's clock (so the safety supervisor's feedback age works with a simulated
clock too) and kept per arbitration ID.
"""
from __future__ import annotations

import time
from collections import defaultdict, deque


def parse_spec(spec: str):
    """'socketcan:can0' → (interface, channel, kwargs)."""
    kind, _, rest = spec.partition(":")
    kind = kind.strip().lower()
    if kind not in ("virtual", "socketcan", "slcan", "gs_usb", "pcan", "kvaser"):
        raise ValueError(f"unknown bus kind {kind!r} (virtual, socketcan, slcan, gs_usb, pcan, kvaser)")
    chan, _, br = rest.partition("@")
    kw = {}
    if br:
        kw["bitrate"] = int(br)
    elif kind in ("slcan", "gs_usb", "pcan", "kvaser"):
        kw["bitrate"] = 1_000_000
    if kind == "gs_usb":
        kw["index"] = int(chan or 0)
        chan = None
    return kind, chan or None, kw


class CanBus:
    def __init__(self, spec: str, clock=time.monotonic, maxlen=64):
        import can
        self.can = can
        self.spec = spec
        kind, chan, kw = parse_spec(spec)
        self.bus = can.Bus(interface=kind, channel=chan, **kw)
        self.clock = clock
        self.inbox = defaultdict(lambda: deque(maxlen=maxlen))     # arb id → deque of (stamp, bytes)
        self.n_tx = self.n_rx = 0
        self.pump = None            # virtual bus: the simulated motors' frame handler, run before each read
        self.err_source = None      # virtual bus: the impairment model's error counters (sw/bus_sim.py)
        self.kind, self.chan = kind, chan

    def send(self, arb_id: int, data: bytes):
        self.bus.send(self.can.Message(arbitration_id=int(arb_id), data=bytes(data), is_extended_id=False))
        self.n_tx += 1

    def drain(self):
        if self.pump is not None:
            self.pump()
        while True:
            m = self.bus.recv(0.0)
            if m is None:
                return
            if m.is_error_frame or m.is_remote_frame:
                continue
            now = self.clock()
            if self.kind != "virtual" and getattr(m, "timestamp", None):
                # kernel RX timestamp (epoch): age the frame from when it arrived, not from when this loop read it, so a
                # frame that sat in the socket buffer during a pause is not taken as fresh
                now -= max(0.0, time.time() - float(m.timestamp))
            self.inbox[m.arbitration_id].append((now, bytes(m.data)))
            self.n_rx += 1

    def recv_for(self, arb_id: int, match=None, timeout: float = 0.0, with_stamp=False):
        """match=None: the newest frame on arb_id (older ones are dropped: a control loop wants the latest).
        match=callable(data) → bool: the oldest matching frame (register replies); non-matching frames stay queued."""
        t_end = time.monotonic() + timeout
        while True:
            self.drain()
            q = self.inbox.get(arb_id)
            if q:
                if match is None:
                    st, d = q[-1]
                    q.clear()
                    return (d, st) if with_stamp else d
                for k, (st, d) in enumerate(q):
                    if match(d):
                        del q[k]
                        return (d, st) if with_stamp else d
            if time.monotonic() >= t_end:
                return (None, None) if with_stamp else None
            time.sleep(0.0002)

    def error_counters(self):
        """CAN controller health: state (ERROR-ACTIVE / -WARNING / -PASSIVE / BUS-OFF), TX/RX error counters, bus-off
        events and restarts, bus errors, dropped frames. SocketCAN: `ip -details -statistics -json link show <if>`;
        virtual: the impairment model; other adapters: None (not reported)."""
        if self.err_source is not None:
            return dict(self.err_source())
        if self.kind != "socketcan":
            return None
        return socketcan_counters(self.chan)

    def ids_seen(self):
        self.drain()
        return sorted(k for k, q in self.inbox.items() if q)

    def clear(self):
        self.drain()
        self.inbox.clear()

    def close(self):
        try:
            self.bus.shutdown()
        except Exception:
            pass


def socketcan_counters(ifname):
    """Parse `ip -details -statistics -json link show <ifname>` (iproute2 ≥ 5.x); None if unavailable."""
    import json
    import subprocess
    try:
        out = subprocess.run(["ip", "-details", "-statistics", "-json", "link", "show", ifname], capture_output=True, text=True, timeout=2)
        js = json.loads(out.stdout)[0]
    except Exception:
        return None
    info = (js.get("linkinfo") or {}).get("info_data") or {}
    xs = (js.get("linkinfo") or {}).get("info_xstats") or {}
    be = info.get("berr_counter") or {}
    st = js.get("stats64") or js.get("stats") or {}
    rx, tx = st.get("rx") or {}, st.get("tx") or {}
    return dict(state=info.get("state"), berr_tx=be.get("tx"), berr_rx=be.get("rx"), bus_off=xs.get("bus_off"),
                restarts=xs.get("restarts"), bus_errors=xs.get("bus_error"), error_passive=xs.get("error_passive"),
                arbitration_lost=xs.get("arbitration_lost"), rx_dropped=rx.get("dropped"), rx_over=rx.get("over_errors"),
                tx_dropped=tx.get("dropped"), rx_errors=rx.get("errors"), tx_errors=tx.get("errors"))
