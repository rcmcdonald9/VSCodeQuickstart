"""SteadyWin GIM6010-8 (SDC104 driver) over CAN: interface only (S1).

The protocol is in the SteadyWin manual (rnd/specs/actuators.yaml source 1), which this environment's network policy
blocks. Fill `encode_*` / `decode_feedback` from the manual on day 1 of M1; everything above this layer (the bus, the
safety supervisor, the 500 Hz loop, the twin) talks to `JointDriver` only, so nothing else changes.

Day-1 checklist (with the motor on the bench, PSU current-limited to 1 A, regen clamp fitted — electrical.md A5):
1. Read the node ID, firmware version and CAN bit rate with the vendor tool over USB-C; set 1 Mbps and unique IDs
   (shoulder bus B1: pitch 1, roll 2, yaw 3; bus B2: elbow 4, DM-J4310 wrist 5).
2. Check whether the SDC104 firmware offers an MIT-style impedance mode (kp, kd, feed-forward torque). If it does,
   `sw/drivers/mit.py` may be reusable with this motor's ranges; if not, use its position mode with a torque limit.
3. Measure the torque constant against a luggage scale on a 0.2 m lever (thermal.md assumes 75 % of nominal in the
   worst corner) and the encoder resolution at the output.
"""
from __future__ import annotations


class JointDriver:
    """One actuator behind a bus. Units at the joint output after the belt ratio is applied by the caller: rad, rad/s, N·m."""

    def __init__(self, bus, can_id: int, ratio: float = 1.0):
        self.bus, self.id, self.ratio = bus, can_id, ratio
        self.state = dict(p=0.0, v=0.0, t=0.0, temp=None, err=0)

    def enable(self):
        raise NotImplementedError("GIM6010 protocol: fill from the SteadyWin manual (see module docstring)")

    def disable(self):
        raise NotImplementedError

    def command(self, p, v, kp, kd, t_ff):
        raise NotImplementedError

    def poll(self):
        raise NotImplementedError


class DMJ4310(JointDriver):
    """Damiao DM-J4310 in MIT mode (sw/drivers/mit.py). Command id = can_id, feedback on the master id."""

    def __init__(self, bus, can_id: int, ratio: float = 1.0, ranges=None):
        from .mit import Ranges
        super().__init__(bus, can_id, ratio)
        self.r = ranges or Ranges()

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
        msg = self.bus.recv_for(self.id)
        if msg is not None:
            fb = unpack_feedback(msg, self.r)
            g = self.ratio
            self.state = dict(p=fb["p"] / g, v=fb["v"] / g, t=fb["t"] * g, temp=max(fb["t_mos"], fb["t_rotor"]), err=fb["err"])
        return self.state
