"""SteadyWin GIM6010-8 (SDC104 driver) over CAN: interface + the protocol TODO table (S1).

The protocol is in the SteadyWin manual (rnd/specs/actuators.yaml source 1), which this environment's network policy
blocks. Everything above this layer (sw/bringup.py, the safety supervisor, the 500 Hz loop, the twin) talks to
`JointDriver` / `sw.drivers.protocol.TableDriver` only, and the GIM is driven from the `GIM6010` table below: **day 1
only fills in the TODO fields** (`python -m sw.drivers.gim` prints what is still missing). Until then sw/bringup.py
lists the GIM joints as BLOCKED and brings up the DM-J4310 wrist alone. The structure follows the DM-J4310 table in
sw/drivers/mit.py (filled), which is the reference for every field; the SDC104's own frames may look different, which
the templates allow (any 8-byte layout with {id_lo} {id_hi} {reg} {v0..v3} placeholders, any arbitration-ID expression).

What is publicly known (2026-11-27 search; the maker's manual host is blocked here) is in `GIM_KNOWN` / `GIM_DRIVERS`
below and in docs/sw/gim_protocol_notes.md: the GIM6010-8 manual rev 1.2 has a CAN protocol section and a release note
"fixed bug in CAN MIT protocol" (2024-05-24), and the GIM GUI manual v2.0 lists position, speed, current and MIT control
modes; but a GIM8115-9 owner reports the MIT frames did nothing until a mode was changed over the serial terminal, and a
GIM8108 manual names the default CAN protocol "SmartCAN". Nothing public covers the SDC303 (GIM8108-36) or says the three
drivers in this arm share one protocol (Fable review M4).

Day-1 checklist (Step 1a of the M1 kit, docs/M1_RIG.md: ONE GIM6010-8 on the rig's motor plate, hub + index disc only,
the 10 A supply at 24.0 V limited to 1 A, NO safety board needed yet; Fable review 2026-11-28 M1):
1. **Protocol discovery**: `python -m sw.gim_discover --bus socketcan:can0 --id <n>`: listen, a no-torque probe
   (MIT exit-motor-mode), then with `--enable` a zero-gain motor-mode trace while you turn the output by hand. Report:
   docs/sw/gim_discovery_<date>.md. Before it: read the node ID, firmware version and bit rate with the vendor tool over
   USB-C, select the MIT control mode there if the tool offers it, set 1 Mbps.
2. If MIT answers: set CODEC "mit", CONTROL / FEEDBACK / ENABLE / DISABLE from the discovery report and RANGES from the
   tool. If it does not: the table cannot be used as is (a position/velocity-only drive needs a different torque bound
   in sw/safety.py): say so in the report and stop. No further GIM is bought until this step passes (M1_READINESS.md).
3. Registers from the manual / vendor tool: IDs, CAN timeout, the current or torque limit and its flash save; prove each
   on the bare motor with `python -m sw.rig_1a --bus socketcan:can0 --joint yaw limits verify impedance timeout stall`
   (written, read back, kept over a power cycle, the MIT law, the drive disables itself after the CAN timeout, the limit
   holds at stall against the luggage scale on the 100 mm stall arm: the torque constant's first check).
4. `python -m sw.drivers.gim` (no TODO left); with the safety board (Step 1b) and the lever rig:
   `python -m sw.bringup --rig step1 --bus socketcan:can0,can1 --estop gpiod:gpiochip0:17:27 --joints yaw,wpitch`.
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

# What is publicly known per field (2026-11-27). status: "candidate" = a documented value of the MIT / Ben Katz family that
# sw/gim_discover.py tests; "unknown" = needs the manual or the vendor tool; "assumed" = what sw/ code assumes today.
GIM_KNOWN = {
    "CODEC": ("candidate", "MIT mode exists in the GIM6010-8 manual rev 1.2 (release note: 'fixed bug in CAN MIT protocol', 2024-05-24) "
                           "and in the GIM GUI v2.0 mode list (position / speed / current / MIT); may have to be selected in the tool first"),
    "ENABLE": ("candidate", "MIT / Ben Katz family: FF FF FF FF FF FF FF FC on the node ID. A GIM8115-9 owner quotes 0x91 00..00 from "
                            "another document, and no motion until a firmware setting was changed over serial"),
    "DISABLE": ("candidate", "FF x7 FD on the node ID (Ben Katz 'exit motor mode'); the discovery probe uses it because it gives no torque"),
    "SET_ZERO": ("candidate", "FF x7 FE (Ben Katz)"),
    "CONTROL": ("candidate", "MIT 8-byte frame on the node ID"),
    "FEEDBACK": ("unknown", "Ben Katz firmware answers on ID 0 with the node ID in byte 0; SteadyWin may use a settable master ID"),
    "RANGES": ("unknown", "P/V/T ranges are firmware settings; candidates tried: P 12.5 / 95.5, V 30 / 45 / 65, T 10 / 18 / 54"),
    "REG.esc_id": ("unknown", "IDs 1-252, unique per bus (GIM GUI v2.0); set in the vendor tool"),
    "REG.can_timeout": ("unknown", "no public source for a CAN-command timeout on SteadyWin drivers"),
    "TORQUE_LIMIT_REG": ("unknown", "no public source for a current / torque limit saved to flash"),
    "STATUS_OK": ("unknown", "no public status / error code list"),
    "ABSOLUTE": ("assumed False", "driver encoders are MT6816 / MA732 class on the motor side (reseller listing): after a power cycle the "
                                  "output angle is known only within one motor turn / 8; the AS5048A joint encoders or a re-zero fix it"),
    "bit rate": ("candidate", "1 Mbit/s (GIM8108 manual: 'SmartCAN protocol, speed 1M')"),
}
GIM_DRIVERS = {   # the three driver types this arm would mix (Fable M4)
    "SDC104 (GDS68)": dict(motor="GIM6010-8 (pitch, yaw, elbow; Step 1)", bus="CAN only, to 48 V / 15 A (reseller)", mit="manual rev 1.2 mentions MIT",
                           status="Step 1 discovers it"),
    "SDC303 (GDM810 / GDK810)": dict(motor="GIM8108-36 (inline roll)", bus="CAN, 24 V version (store listing)", mit="no public document",
                                     status="seller question 5; discovery again on arrival (Step 2)"),
    "GIM8108-8 driver (SDC104 / GDZ468 = SDC215 / GDS68?)": dict(motor="GIM8108-8 (hold A)", bus="GDZ468: CAN + RS-485, 12-40 V, 10 A (reseller)",
                                                                 mit="unknown", status="seller questions 1 + 5; hold A"),
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
        st, why = GIM_KNOWN.get(k, ("unknown", ""))
        print(f"  TODO {k:22s} {GIM_TODO_HELP.get(k, '')}  [{st}] {why}")
    raise SystemExit(1 if m else 0)
