"""MIT-mode CAN frames (S1). The Damiao DM-J4310 speaks this format ("MIT mode"); it is the MIT Mini Cheetah layout.

Command (8 bytes): position 16 bit | velocity 12 | kp 12 | kd 12 | torque 12, each a linear map of a float range.
Feedback (8 bytes): byte 0 = id (low nibble) | error (high nibble); position 16 | velocity 12 | torque 12; MOS °C; rotor °C.
Special frames: enable FF×7 FC, disable FF×7 FD, set zero FF×7 FE.

The float ranges (P_MAX, V_MAX, T_MAX) are set per motor in Damiao's config tool; the defaults below are the J4310
factory values as commonly documented (ESTIMATE: read them back from the motor on day 1 and put them in joints.json).
The GIM6010-8 (SteadyWin SDC104) protocol is not implemented: its manual could not be fetched from here. `gim.py`
holds the TODO table to fill from the manual when the motors arrive. `DM_J4310` below is the full protocol table
(IDs, special frames, registers) that sw/drivers/protocol.py `TableDriver` and sw/bringup.py use.
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


# ---- DM-J4310 protocol table (sw/drivers/protocol.py TableDriver). Source: Damiao DM-J4310 user manual (CAN protocol
# section) and Damiao's open-source host library (DM_CAN.py: register list `DM_variable`, 0x7FF parameter frames).
# Items marked VERIFY are checked by sw/bringup.py on day 1 (the step fails if the drive disagrees).
DM_J4310 = {
    "NAME": "Damiao DM-J4310-2EC V1.1 (MIT mode)",
    "CONTROL": "{id}",                                         # MIT frame on the command ID (ESC_ID)
    "FEEDBACK": "{fb}",                                        # every reply on the master ID (MST_ID)
    "CODEC": "mit",
    "RANGES": dict(p_max=12.5, v_max=30.0, t_max=10.0, kp_max=500.0, kd_max=5.0),   # factory; re-read from the drive
    "RANGE_REGS": {"p_max": "p_max", "v_max": "v_max", "t_max": "t_max"},
    "ENABLE": ("{id}", "FF FF FF FF FF FF FF FC"),
    "DISABLE": ("{id}", "FF FF FF FF FF FF FF FD"),
    "SET_ZERO": ("{id}", "FF FF FF FF FF FF FF FE"),
    "CLEAR_ERROR": ("{id}", "FF FF FF FF FF FF FF FB"),
    "REFRESH": ("0x7FF", "{id_lo} {id_hi} CC 00 00 00 00 00"),               # status request → one feedback frame (VERIFY)
    "PARAM_READ": ("0x7FF", "{id_lo} {id_hi} 33 {reg} 00 00 00 00"),
    "PARAM_WRITE": ("0x7FF", "{id_lo} {id_hi} 55 {reg} {v0} {v1} {v2} {v3}"),  # RAM; the drive echoes it
    "PARAM_SAVE": ("0x7FF", "{id_lo} {id_hi} AA 01 00 00 00 00"),              # to flash; only while disabled
    "PARAM_REPLY": ("{fb}", "{id_lo} {id_hi} * {reg} {v0} {v1} {v2} {v3}"),
    "PARAM_REPLY_OPS": (0x33, 0x55), "PARAM_REPLY_OP_BYTE": 2,
    "FB_ID_NIBBLE": True,                                      # feedback byte 0 low nibble = command ID
    # feedback byte 0 high nibble: 0 disabled, 1 enabled, 8 over-voltage, 9 under-voltage, A over-current,
    # B MOS over-temperature, C rotor over-temperature, D communication lost (CAN timeout), E overload
    "STATUS_OK": (0, 1), "STATUS_ENABLED": 1, "STATUS_DISABLED": 0, "STATUS_TIMEOUT": 0xD,
    "STATUS_NAMES": {0: "disabled", 1: "enabled", 8: "over-voltage", 9: "under-voltage", 10: "over-current",
                     11: "MOS over-temperature", 12: "rotor over-temperature", 13: "communication lost", 14: "overload"},
    "REG": {"kt": (1, "f32"), "mst_id": (7, "u32"), "esc_id": (8, "u32"), "can_timeout": (9, "u32"),
            "ctrl_mode": (10, "u32"), "hw_version": (13, "u32"), "fw_version": (14, "u32"), "serial": (15, "u32"),
            "gear_ratio": (20, "f32"), "p_max": (21, "f32"), "v_max": (22, "f32"), "t_max": (23, "f32")},
    "CTRL_MODE_MIT": 1,                                        # CTRL_MODE: 1 MIT, 2 position-velocity, 3 velocity, 4 force-position
    # Drive-side torque bound: in MIT mode the drive maps and clamps torque to ±T_MAX (VERIFY on day 1: the calibration
    # step pushes past it against the hard stop at a low limit and checks that the reported torque saturates).
    "TORQUE_LIMIT_REG": "t_max", "TORQUE_LIMIT_UNIT": "Nm",
    "CAN_TIMEOUT_UNIT_S": 50e-6,                               # TIMEOUT register counts 50 µs periods (VERIFY: step 3 measures it)
    "DEFAULT_ID": (0x01, 0x00),                                # factory ESC_ID / MST_ID
    "ABSOLUTE": True,                                          # 2EC: output-side absolute encoder, the zero survives a power cycle
}
