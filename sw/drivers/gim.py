"""SteadyWin GIM6010-8 (SDC104 driver) over CAN: interface + the protocol TODO table (S1).

The protocol is in the SteadyWin manual (rnd/specs/actuators.yaml source 1), which this environment's network policy
blocks. Everything above this layer (sw/bringup.py, the safety supervisor, the 500 Hz loop, the twin) talks to
`JointDriver` / `sw.drivers.protocol.TableDriver` only, and the GIM is driven from the `GIM6010` table below: **day 1
only fills in the TODO fields** (`python -m sw.drivers.gim` prints what is still missing). Until then sw/bringup.py
lists the GIM joints as BLOCKED and brings up the DM-J4310 wrist alone. The structure follows the DM-J4310 table in
sw/drivers/mit.py (filled), which is the reference for every field; the SDC104's own frames may look different, which
the templates allow (any 8-byte layout with {id_lo} {id_hi} {reg} {v0..v3} placeholders, any arbitration-ID expression).

Day-1 checklist (with the motor on the bench, PSU current-limited to 1 A, regen clamp fitted — electrical.md A5):
1. Read the node ID, firmware version and CAN bit rate with the vendor tool over USB-C; set 1 Mbps and unique IDs
   (shoulder bus B1: pitch 1, roll 2, yaw 3; bus B2: elbow 4, DM-J4310 wrist 5; feedback IDs +0x10, can_map.yaml).
2. Check whether the SDC104 firmware offers an MIT-style impedance mode (kp, kd, feed-forward torque). If it does, set
   CODEC "mit" and its ranges; if not, the table cannot be used as is (position mode needs a different torque bound in
   sw/safety.py: say so in the bring-up report and stop).
3. Measure the torque constant against a luggage scale on a 0.2 m lever (thermal.md assumes 75 % of nominal in the
   worst corner) and the encoder resolution at the output.
4. Fill the table, run `python -m sw.drivers.gim` (no TODO left), then `python -m sw.bringup --bus socketcan:can0,can1`.
"""
from __future__ import annotations

TODO = None

# ---------------------------------------------------------------------------------------------------------------------
# GIM6010 protocol table — FILL ON DAY 1 from the SteadyWin SDC104 CAN manual. Each TODO names what to look up.
# ---------------------------------------------------------------------------------------------------------------------
GIM6010 = {
    "NAME": "SteadyWin GIM6010-8, SDC104 driver",
    "CONTROL": TODO,          # arbitration ID of the impedance/MIT control frame, e.g. "{id}" or "0x100+{id}"
    "FEEDBACK": TODO,         # arbitration ID the drive answers on, e.g. "{fb}" (a settable master ID) or "0x580+{id}"
    "CODEC": TODO,            # "mit" if the 8-byte control/feedback frames are the MIT layout (sw/drivers/mit.py)
    "RANGES": TODO,           # MIT float ranges at the motor output: dict(p_max=, v_max=, t_max=, kp_max=, kd_max=)
    "RANGE_REGS": {},         # registers holding those ranges, if the drive exposes them: {"p_max": "<REG name>", ...}
    "ENABLE": TODO,           # (arb, "8 bytes"): enter closed loop / motor on
    "DISABLE": TODO,          # (arb, "8 bytes"): motor off (idle, no torque)
    "SET_ZERO": TODO,         # (arb, "8 bytes"): set the current position as zero (optional: bring-up keeps the zero in software)
    "CLEAR_ERROR": TODO,      # (arb, "8 bytes"): clear a latched drive fault
    "REFRESH": TODO,          # (arb, "8 bytes"): request one feedback frame without commanding motion
    "PARAM_READ": TODO,       # (arb, "8 bytes with {reg}"): read a parameter
    "PARAM_WRITE": TODO,      # (arb, "8 bytes with {reg} {v0}..{v3}"): write a parameter (RAM)
    "PARAM_SAVE": TODO,       # (arb, "8 bytes"): store parameters to flash
    "PARAM_REPLY": TODO,      # (arb, "reply pattern with {reg} {v0}..{v3}, * = any byte"): the drive's answer to read/write
    "PARAM_REPLY_OPS": (), "PARAM_REPLY_OP_BYTE": -1,   # opcode byte values that mark a reply (helps tell it from feedback)
    "FB_ID_NIBBLE": TODO,     # True if feedback byte 0 low nibble carries the node ID (MIT layout)
    "STATUS_OK": TODO,        # feedback status codes that are healthy, e.g. (0, 1)
    "STATUS_ENABLED": TODO,   # status code while closed loop
    "STATUS_DISABLED": TODO,  # status code while idle
    "STATUS_TIMEOUT": TODO,   # status code after the drive's CAN timeout tripped (if it reports one)
    "STATUS_NAMES": {},       # code → text, for the report
    "REG": {                  # register numbers and formats ("u32" / "f32", little endian)
        "fw_version": TODO,   # firmware version
        "esc_id": TODO,       # node (command) ID
        "mst_id": TODO,       # feedback / master ID
        "ctrl_mode": TODO,    # control mode (must read CTRL_MODE_MIT)
        "can_timeout": TODO,  # CAN-command timeout (drive disables itself when commands stop)
        "t_max": TODO,        # the register that bounds output torque or current (see TORQUE_LIMIT_*)
    },
    "CTRL_MODE_MIT": TODO,    # value of ctrl_mode for the impedance mode
    "TORQUE_LIMIT_REG": TODO,  # name in REG of the current/torque limit, e.g. "t_max" or "i_max"
    "TORQUE_LIMIT_UNIT": TODO,  # "Nm" (motor output) or "A" (phase current; then KT_NM_PER_A is needed)
    "KT_NM_PER_A": TODO,      # torque constant at the GIM output (after its 8:1 gear), N·m/A: measure (checklist 3)
    "CAN_TIMEOUT_UNIT_S": TODO,  # seconds per count of the can_timeout register
    "DEFAULT_ID": TODO,       # factory (command ID, feedback ID), to detect unconfigured motors
    "ABSOLUTE": TODO,         # True if the output position survives a power cycle (else re-zero, or use the AS5048A joint encoders)
}

# What each field means for the report / TODO printout
GIM_TODO_HELP = {
    "CONTROL": "arbitration ID of the control frame", "FEEDBACK": "arbitration ID of the drive's replies",
    "CODEC": "'mit' if the control/feedback frames are the MIT 8-byte layout", "RANGES": "MIT float ranges (p, v, τ, kp, kd)",
    "ENABLE": "enable frame", "DISABLE": "disable frame", "CLEAR_ERROR": "clear-fault frame", "REFRESH": "status-request frame",
    "PARAM_READ": "parameter read frame", "PARAM_WRITE": "parameter write frame", "PARAM_SAVE": "save-to-flash frame",
    "PARAM_REPLY": "parameter reply pattern", "STATUS_OK": "healthy status codes", "STATUS_ENABLED": "status code when enabled",
    "REG.fw_version": "firmware-version register", "REG.esc_id": "node-ID register", "REG.mst_id": "feedback-ID register",
    "REG.can_timeout": "CAN-timeout register", "TORQUE_LIMIT_REG": "current/torque-limit register",
    "TORQUE_LIMIT_UNIT": "unit of that register (Nm or A)", "CAN_TIMEOUT_UNIT_S": "seconds per CAN-timeout count",
}


class JointDriver:
    """One actuator behind a bus. Units at the joint output after the belt ratio is applied by the caller: rad, rad/s, N·m."""

    def __init__(self, bus, can_id: int, ratio: float = 1.0):
        self.bus, self.id, self.ratio = bus, can_id, ratio
        self.state = dict(p=0.0, v=0.0, t=0.0, temp=None, err=0)

    def enable(self):
        raise NotImplementedError("GIM6010 protocol: fill the GIM6010 table in sw/drivers/gim.py from the SteadyWin manual")

    def disable(self):
        raise NotImplementedError

    def command(self, p, v, kp, kd, t_ff):
        raise NotImplementedError

    def poll(self):
        raise NotImplementedError


def gim_driver(bus, name, can_id, fb_id, ratio, table=None, **kw):
    """A GIM6010 joint: the table-driven driver with the GIM table (raises ProtocolTodo on use while fields are TODO)."""
    from .protocol import TableDriver
    return TableDriver(bus, name, can_id, fb_id, ratio, table or GIM6010, **kw)


class DMJ4310(JointDriver):
    """Damiao DM-J4310 in MIT mode (sw/drivers/mit.py), minimal: command id = can_id, feedback read on `fb_id`
    (the master ID; default can_id for the simple test bus). sw/bringup.py uses the full TableDriver with mit.DM_J4310."""

    def __init__(self, bus, can_id: int, ratio: float = 1.0, ranges=None, fb_id=None):
        from .mit import Ranges
        super().__init__(bus, can_id, ratio)
        self.r = ranges or Ranges()
        self.fb = can_id if fb_id is None else fb_id

    def enable(self):
        from .mit import ENABLE
        self.bus.send(self.id, ENABLE)

    def disable(self):
        from .mit import DISABLE
        self.bus.send(self.id, DISABLE)

    def command(self, p, v, kp, kd, t_ff):
        from .mit import pack_cmd
        g = self.ratio
        self.bus.send(self.id, pack_cmd(p * g, v * g, kp / g ** 2, kd / g ** 2, t_ff / g, self.r))

    def poll(self):
        from .mit import unpack_feedback
        msg = self.bus.recv_for(self.fb)
        if msg is not None:
            fb = unpack_feedback(msg, self.r)
            g = self.ratio
            # Damiao status nibble: 0 disabled and 1 enabled are healthy, 8–14 are faults (mit.DM_J4310 STATUS_NAMES). The
            # supervisor treats any err != 0 as a fault, so only the fault code is passed as err.
            self.state = dict(p=fb["p"] / g, v=fb["v"] / g, t=fb["t"] * g, temp=max(fb["t_mos"], fb["t_rotor"]),
                              err=0 if fb["err"] in (0, 1) else fb["err"], status=fb["err"])
        return self.state


if __name__ == "__main__":
    from .protocol import missing
    m = missing(GIM6010)
    print(f"GIM6010 table: {len(m)} field(s) still TODO" if m else "GIM6010 table complete ✅")
    for k in m:
        print(f"  TODO {k:22s} {GIM_TODO_HELP.get(k, '')}")
    raise SystemExit(1 if m else 0)
