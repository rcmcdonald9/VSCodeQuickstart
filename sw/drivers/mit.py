"""MIT-mode CAN frames (S1). The Damiao DM-J4310 speaks this format ("MIT mode"); it is the MIT Mini Cheetah layout.

Command (8 bytes): position 16 bit | velocity 12 | kp 12 | kd 12 | torque 12, each a linear map of a float range.
Feedback (8 bytes): byte 0 = id (low nibble) | error (high nibble); position 16 | velocity 12 | torque 12; MOS °C; rotor °C.
Special frames: enable FF×7 FC, disable FF×7 FD, set zero FF×7 FE.

The float ranges (P_MAX, V_MAX, T_MAX) are set per motor in Damiao's config tool; the defaults below are the J4310
factory values as commonly documented (ESTIMATE: read them back from the motor on day 1 and put them in joints.json).
The GIM6010-8 (SteadyWin SDC104) protocol is not implemented: its manual could not be fetched from here. `gim.py`
holds the interface to fill from the manual when the motors arrive.
"""
from __future__ import annotations

from dataclasses import dataclass

ENABLE = bytes([0xFF] * 7 + [0xFC])
DISABLE = bytes([0xFF] * 7 + [0xFD])
ZERO = bytes([0xFF] * 7 + [0xFE])


@dataclass(frozen=True)
class Ranges:
    p_max: float = 12.5     # rad
    v_max: float = 30.0     # rad/s
    t_max: float = 10.0     # N·m
    kp_max: float = 500.0
    kd_max: float = 5.0


def f2u(x, lo, hi, bits):
    x = min(max(x, lo), hi)
    return int(round((x - lo) / (hi - lo) * ((1 << bits) - 1)))


def u2f(u, lo, hi, bits):
    return u / ((1 << bits) - 1) * (hi - lo) + lo


def pack_cmd(p, v, kp, kd, t, r: Ranges = Ranges()) -> bytes:
    P = f2u(p, -r.p_max, r.p_max, 16)
    V = f2u(v, -r.v_max, r.v_max, 12)
    KP = f2u(kp, 0, r.kp_max, 12)
    KD = f2u(kd, 0, r.kd_max, 12)
    T = f2u(t, -r.t_max, r.t_max, 12)
    return bytes([P >> 8, P & 0xFF, V >> 4, ((V & 0xF) << 4) | (KP >> 8), KP & 0xFF, KD >> 4, ((KD & 0xF) << 4) | (T >> 8), T & 0xFF])


def unpack_cmd(b: bytes, r: Ranges = Ranges()):
    P = (b[0] << 8) | b[1]
    V = (b[2] << 4) | (b[3] >> 4)
    KP = ((b[3] & 0xF) << 8) | b[4]
    KD = (b[5] << 4) | (b[6] >> 4)
    T = ((b[6] & 0xF) << 8) | b[7]
    return (u2f(P, -r.p_max, r.p_max, 16), u2f(V, -r.v_max, r.v_max, 12), u2f(KP, 0, r.kp_max, 12),
            u2f(KD, 0, r.kd_max, 12), u2f(T, -r.t_max, r.t_max, 12))


def pack_feedback(mid, err, p, v, t, t_mos=30, t_rotor=30, r: Ranges = Ranges()) -> bytes:
    """For the virtual bus / tests: what a motor would send back."""
    P = f2u(p, -r.p_max, r.p_max, 16)
    V = f2u(v, -r.v_max, r.v_max, 12)
    T = f2u(t, -r.t_max, r.t_max, 12)
    return bytes([(err << 4) | (mid & 0xF), P >> 8, P & 0xFF, V >> 4, ((V & 0xF) << 4) | (T >> 8), T & 0xFF, t_mos, t_rotor])


def unpack_feedback(b: bytes, r: Ranges = Ranges()):
    mid, err = b[0] & 0xF, b[0] >> 4
    P = (b[1] << 8) | b[2]
    V = (b[3] << 4) | (b[4] >> 4)
    T = ((b[4] & 0xF) << 8) | b[5]
    return dict(id=mid, err=err, p=u2f(P, -r.p_max, r.p_max, 16), v=u2f(V, -r.v_max, r.v_max, 12),
                t=u2f(T, -r.t_max, r.t_max, 12), t_mos=b[6], t_rotor=b[7])
