"""Table-driven CAN actuator driver (S1): one driver class, one protocol table per motor family.

A protocol table is plain data: arbitration-ID expressions, 8-byte frame templates, register numbers and units. The
DM-J4310 table (sw/drivers/mit.py `DM_J4310`) is filled; the GIM6010 table (sw/drivers/gim.py `GIM6010`) holds TODOs
until the SteadyWin manual is in hand. `TableDriver` refuses to talk to a motor whose table still has a TODO in a field
it needs (`missing()` lists them), so a half-filled table fails loudly at enumeration, not halfway through a move.

Template syntax (8 tokens, space separated): a hex byte ("FF"), a placeholder ({id_lo} {id_hi} = command ID little
endian, {reg} = register number, {v0}..{v3} = the 4-byte register value) or "*" (any byte; only in reply patterns).
Arbitration-ID expressions: "{id}" (command ID from the CAN map), "{fb}" (feedback/master ID), hex/decimal constants and
"+", e.g. "0x7FF", "0x100+{id}".

Units at the joint output (after belt ratio and direction sign): rad, rad/s, N·m. The motor side is what the drive
sees. q_joint = sign · p_motor / ratio − offset (offset = the calibrated zero, set by sw/bringup.py).
"""
from __future__ import annotations

import struct
import time

from .gim import JointDriver
from .mit import Ranges, pack_cmd, unpack_feedback

TODO = None

# Fields a driver needs before it may talk to a motor, by use
NEEDED = {
    "identify": ("CONTROL", "FEEDBACK", "PARAM_READ", "PARAM_REPLY", "REG.fw_version", "REG.esc_id", "REG.mst_id", "STATUS_OK"),
    "limits": ("PARAM_WRITE", "PARAM_SAVE", "TORQUE_LIMIT_REG", "TORQUE_LIMIT_UNIT", "REG.can_timeout", "CAN_TIMEOUT_UNIT_S"),
    "motion": ("CODEC", "RANGES", "ENABLE", "DISABLE", "CLEAR_ERROR", "REFRESH", "STATUS_ENABLED"),
}


# Frame plausibility gate (TableDriver.poll), joint-side units: rad/s, rad/s², s, rad, rad/s, °C. v_max is twice the
# supervisor's V_FAULT; slack_s covers frames bunched by bus jitter (two samples received in one cycle). Sized so that a
# 2 ms frame spacing passes any physical motion of these joints (wrist pitch: 7 N·m peak / 0.02 kg·m² ≈ 350 rad/s² →
# 0.7 rad/s per frame, under the 2.3 rad/s bound) while a flipped high bit in position (≥ bit 9, 0.2 rad), velocity
# (≥ bit 8, 3.7 rad/s) or a temperature byte is dropped.
# dtau_frac: a torque step larger than this fraction of the frame's torque range (= the drive's limit, ±T_MAX) in one
# frame is rejected (a flipped bit ≥ 10 of the 12-bit field); smaller corruptions stay under the collision threshold.
GATE = dict(v_max=8.0, a_max=150.0, slack_s=0.010, dp0=0.02, dv0=0.5, dtemp=5.0, dtau_frac=0.3, n_reanchor=3)


class ProtocolTodo(RuntimeError):
    pass


def get(table, key):
    if key.startswith("REG."):
        return (table.get("REG") or {}).get(key[4:], TODO)
    return table.get(key, TODO)


def missing(table, uses=("identify", "limits", "motion")):
    out = []
    for u in uses:
        for k in NEEDED[u]:
            v = get(table, k)
            if v is TODO or (isinstance(v, tuple) and any(x is TODO for x in v)):
                out.append(k)
    tl = get(table, "TORQUE_LIMIT_REG")
    if "limits" in uses and tl is not TODO and get(table, f"REG.{tl}") is TODO:
        out.append(f"REG.{tl}")
    if "limits" in uses and get(table, "TORQUE_LIMIT_UNIT") == "A" and get(table, "KT_NM_PER_A") is TODO:
        out.append("KT_NM_PER_A")
    if get(table, "CODEC") not in (TODO, "mit"):
        out.append("CODEC (only 'mit' is implemented: the safety supervisor bounds kp·e + kd·ė + τ_ff)")
    return out


def arb(expr, can_id, fb_id):
    total = 0
    for term in str(expr).replace("{id}", str(can_id)).replace("{fb}", str(fb_id)).split("+"):
        total += int(term.strip(), 0)
    return total


def _value_bytes(value, fmt):
    if value is None:
        return b"\x00\x00\x00\x00"
    return struct.pack("<f", float(value)) if fmt == "f32" else struct.pack("<I", int(value) & 0xFFFFFFFF)


def build(template, can_id, reg=0, value=None, fmt="u32"):
    vb = _value_bytes(value, fmt)
    subs = {"{id_lo}": can_id & 0xFF, "{id_hi}": (can_id >> 8) & 0xFF, "{reg}": reg & 0xFF,
            "{v0}": vb[0], "{v1}": vb[1], "{v2}": vb[2], "{v3}": vb[3]}
    out = []
    for tok in template.split():
        out.append(subs[tok] if tok in subs else int(tok, 16))
    assert len(out) == 8, template
    return bytes(out)


def matcher(template, can_id, reg):
    """A reply pattern → callable(data) → bool. Value bytes and '*' match anything."""
    want = []
    for tok in template.split():
        if tok in ("*", "{v0}", "{v1}", "{v2}", "{v3}"):
            want.append(None)
        elif tok == "{id_lo}":
            want.append(can_id & 0xFF)
        elif tok == "{id_hi}":
            want.append((can_id >> 8) & 0xFF)
        elif tok == "{reg}":
            want.append(reg & 0xFF)
        else:
            want.append(int(tok, 16))
    return lambda d: len(d) == 8 and all(w is None or w == b for w, b in zip(want, d))


def parse(template, data):
    """Inverse of build(): None if a constant byte differs, else {'id': .., 'reg': .., 'value_bytes': ..}."""
    if len(data) != 8:
        return None
    out, vb, lo, hi = {}, [0, 0, 0, 0], None, None
    for tok, b in zip(template.split(), data):
        if tok == "*":
            continue
        if tok == "{id_lo}":
            lo = b
        elif tok == "{id_hi}":
            hi = b
        elif tok == "{reg}":
            out["reg"] = b
        elif tok.startswith("{v"):
            vb[int(tok[2])] = b
        elif int(tok, 16) != b:
            return None
    if lo is not None:
        out["id"] = lo | ((hi or 0) << 8)
    out["value_bytes"] = bytes(vb)
    return out


def decode_value(vb, fmt):
    return struct.unpack("<f", vb)[0] if fmt == "f32" else struct.unpack("<I", vb)[0]


def reply_value(template, data, fmt):
    toks = template.split()
    vb = bytes(data[toks.index(f"{{v{i}}}")] for i in range(4))
    return struct.unpack("<f", vb)[0] if fmt == "f32" else struct.unpack("<I", vb)[0]


class TableDriver(JointDriver):
    """One actuator on a `CanBus` (sw/drivers/canbus.py), speaking the protocol in `table`."""

    def __init__(self, bus, name, can_id, fb_id, ratio, table, sign=1, offset=0.0):
        super().__init__(bus, can_id, ratio)
        self.name, self.fb, self.table, self.sign, self.offset = name, fb_id, table, int(sign), float(offset)
        rg = table.get("RANGES") or {}
        self.r = Ranges(**rg) if rg and TODO not in rg.values() else Ranges()
        self._replies = []            # param replies seen while polling feedback
        self.idle = None              # callable run while waiting for a register reply (one control cycle)
        self.gate = True              # frame plausibility gate (poll)
        self._acc, self._acc_t, self._n_rej, self._last_status = None, None, 0, None
        self.rejected = self.held = 0
        self.stamp = None
        self.state = dict(p=float("nan"), v=float("nan"), t=float("nan"), temp=None, err=0, status=None)

    # ---- plumbing
    def missing(self, uses=("identify", "limits", "motion")):
        return missing(self.table, uses)

    def _need(self, *keys):
        bad = [k for k in keys if get(self.table, k) is TODO]
        if bad:
            raise ProtocolTodo(f"{self.name}: protocol table field(s) {bad} are TODO ({self.table.get('NAME')})")

    def _send(self, key, reg=0, value=None, fmt="u32"):
        self._need(key)
        a, tpl = self.table[key]
        self.bus.send(arb(a, self.id, self.fb), build(tpl, self.id, reg, value, fmt))

    def _fb_arb(self):
        return arb(self.table["FEEDBACK"], self.id, self.fb)

    def _take(self):
        """All frames on the feedback ID since the last call: param replies are stashed, feedback frames returned."""
        self.bus.drain()
        q = self.bus.inbox.get(self._fb_arb())
        frames = list(q) if q else []
        if q:
            q.clear()
        tpl = self.table.get("PARAM_REPLY")
        fbs = []
        for st, d in frames:
            if tpl and tpl[1] and self._is_param_reply(d):
                self._replies.append(d)
            else:
                fbs.append((st, d))
        return fbs

    def _is_param_reply(self, d):
        tpl = self.table["PARAM_REPLY"][1]
        toks = tpl.split()
        # a reply carries our command ID in the {id_lo}/{id_hi} slots and a known opcode where the template has a constant
        ok_ops = self.table.get("PARAM_REPLY_OPS", ())
        for i, tok in enumerate(toks):
            if tok == "{id_lo}" and d[i] != (self.id & 0xFF):
                return False
            if tok == "{id_hi}" and d[i] != ((self.id >> 8) & 0xFF):
                return False
            if tok == "*" and ok_ops and i == self.table.get("PARAM_REPLY_OP_BYTE", -1) and d[i] not in ok_ops:
                return False
        return True

    def set_offset(self, offset):
        """New calibrated zero; the last feedback is re-expressed in it so the loop sees no jump in age or frame."""
        self.state["p"] = self.state["p"] + self.offset - offset
        self.offset = float(offset)

    # ---- special frames
    def enable(self):
        self._send("ENABLE")

    def disable(self):
        self._send("DISABLE")

    def set_zero(self):
        self._send("SET_ZERO")

    def clear_error(self):
        self._send("CLEAR_ERROR")

    def refresh(self):
        self._send("REFRESH")

    # ---- control
    def command(self, p, v, kp, kd, t_ff):
        g, s = self.ratio, self.sign
        self.bus.send(arb(self.table["CONTROL"], self.id, self.fb),
                      pack_cmd(s * g * (p + self.offset), s * g * v, kp / g ** 2, kd / g ** 2, s * t_ff / g, self.r))

    def decode(self, d):
        fb = unpack_feedback(d, self.r)
        g, s = self.ratio, self.sign
        ok = self.table.get("STATUS_OK", (0,))
        return dict(p=s * fb["p"] / g - self.offset, v=s * fb["v"] / g, t=s * fb["t"] * g, temp=max(fb["t_mos"], fb["t_rotor"]),
                    status=fb["err"], err=0 if fb["err"] in ok else fb["err"], id=fb["id"], p_motor=fb["p"], v_motor=fb["v"], t_motor=fb["t"])

    def _plausible(self, new, st):
        """Frame plausibility gate (bus-realism finding, docs/sw/bus_realism.md): a payload that passed the CAN CRC can
        still be corrupt (a buggy controller driver / adapter firmware). Compared with the last accepted frame, motor
        side: a position jump faster than GATE v_max, a velocity jump faster than a_max, a temperature jump > dtemp °C is
        rejected (= a lost frame: the supervisor's stale-feedback check still bounds how many can be lost). A drive error
        status must be seen on two frames in a row. After n_reanchor rejects in a row the newest frame is taken (the
        reference itself was wrong, or the drive was power-cycled and re-zeroed)."""
        G, old = GATE, self._acc
        if not self.gate or old is None:
            return True
        if self._n_rej >= G["n_reanchor"]:
            return True
        dt = max(st - self._acc_t, 0.0) + G["slack_s"]
        r = self.ratio
        if abs(new["p_motor"] - old["p_motor"]) > r * (G["v_max"] * dt + G["dp0"]):
            return False
        if abs(new["v_motor"] - old["v_motor"]) > r * (G["a_max"] * dt + G["dv0"]):
            return False
        if new["temp"] is not None and old["temp"] is not None and abs(new["temp"] - old["temp"]) > G["dtemp"]:
            return False
        if abs(new["t_motor"] - old["t_motor"]) > G["dtau_frac"] * self.r.t_max:
            return False
        return True

    def poll(self):
        """Newest plausible feedback frame (non-blocking); state['fresh'] says whether one arrived since the last poll."""
        fbs = self._take()
        fresh = False
        ok = self.table.get("STATUS_OK", (0,))
        for st, d in fbs:
            if self.table.get("FB_ID_NIBBLE", True) and (d[0] & 0x0F) != (self.id & 0x0F):
                continue
            new = self.decode(d)
            if not self._plausible(new, st):
                self._n_rej += 1
                self.rejected += 1
                continue
            if self.gate and new["status"] not in ok and new["status"] != self._last_status:
                self._last_status = new["status"]           # an error status counts from its second frame
                self.held += 1
                continue
            self._last_status = new["status"]
            self._n_rej = 0
            self._acc, self._acc_t = new, st
            self.state = new
            self.stamp = st
            fresh = True
        self.state["fresh"] = fresh
        return self.state

    def wait_feedback(self, timeout=0.02):
        t_end = time.monotonic() + timeout
        while True:
            s = self.poll()
            if s["fresh"]:
                return s
            if time.monotonic() >= t_end:
                return None
            time.sleep(0.0002)

    # ---- registers
    def _reg(self, name):
        self._need(f"REG.{name}")
        return self.table["REG"][name]

    def _wait_reply(self, rid, timeout):
        """With `self.idle` set (sw/bringup.py does), the wait runs control cycles (heartbeat, hold commands) on the bus
        clock instead of blocking: a register reply that is late or lost then costs cycles, not a heartbeat gap."""
        tpl = self.table["PARAM_REPLY"][1]
        m = matcher(tpl, self.id, rid)
        clock = self.bus.clock if self.idle is not None else time.monotonic
        t_end = clock() + timeout
        while True:
            self._take()
            for k, d in enumerate(self._replies):
                if m(d):
                    del self._replies[k]
                    return d
            if clock() >= t_end:
                return None
            if self.idle is not None:
                self.idle()
            else:
                time.sleep(0.0002)

    def read_reg(self, name, timeout=0.05, retries=3):
        rid, fmt = self._reg(name)
        for _ in range(retries):
            self._send("PARAM_READ", rid)
            d = self._wait_reply(rid, timeout)
            if d is not None:
                return reply_value(self.table["PARAM_REPLY"][1], d, fmt)
        return None

    def write_reg(self, name, value, timeout=0.05, retries=3):
        """Write (RAM) and return the value the drive echoes, or None."""
        rid, fmt = self._reg(name)
        for _ in range(retries):
            self._send("PARAM_WRITE", rid, value, fmt)
            d = self._wait_reply(rid, timeout)
            if d is not None:
                return reply_value(self.table["PARAM_REPLY"][1], d, fmt)
        return None

    def save(self):
        self._send("PARAM_SAVE")

    def ranges_from_drive(self):
        """Re-read the MIT frame ranges (P_MAX, V_MAX, T_MAX) from the drive so the frames are scaled as the drive scales them."""
        rr = self.table.get("RANGE_REGS") or {}
        vals = {}
        for field, reg in rr.items():
            v = self.read_reg(reg)
            if v is None:
                return None
            vals[field] = float(v)
        if vals:
            cur = self.r.__dict__.copy()
            cur.update(vals)
            self.r = Ranges(**cur)
        return vals

    # ---- torque limit in drive units
    def torque_limit_to_drive(self, t_joint):
        """Joint-side N·m → the drive's torque-limit register value."""
        unit = self.table["TORQUE_LIMIT_UNIT"]
        t_m = t_joint / self.ratio
        if unit == "Nm":
            return t_m
        if unit == "A":
            kt = self.table.get("KT_NM_PER_A")
            if kt in (TODO, None):
                raise ProtocolTodo(f"{self.name}: KT_NM_PER_A is TODO")
            return t_m / kt
        raise ProtocolTodo(f"{self.name}: unknown TORQUE_LIMIT_UNIT {unit!r}")

    def torque_limit_from_drive(self, raw):
        unit = self.table["TORQUE_LIMIT_UNIT"]
        t_m = raw if unit == "Nm" else raw * self.table["KT_NM_PER_A"]
        return t_m * self.ratio
