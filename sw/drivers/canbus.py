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
            self.inbox[m.arbitration_id].append((self.clock(), bytes(m.data)))
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
