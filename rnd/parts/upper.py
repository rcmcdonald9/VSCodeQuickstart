"""Mast, lift, torso and head v0 (robot frame: x forward, z up; mast axis at x = MAST_X).

- Mast: 60×60×2 Al square tube from the base coupling to the head bracket (rnd/mast.py).
- Lift: GIM cartridge at the mast foot drives a 20T HTD 5M pulley; the belt loops over an idler in the top bracket and
  clamps to the carriage. Printed ratchet strip on the mast's back face; spring-applied pawl + 12 V solenoid on the
  carriage (rnd/lift.py).
- Carriage: printed sleeve with 8 × 625 bearings on the tube faces; the torso shell and both pitch modules bolt to it.
- Torso: two-piece shell with a chest light; electronics tray (Pi 5, CAN, DC-DC, fuses) on the back; e-stop on top.
- Head (fixed on the mast top, D8): top bracket (idler + carry handle) → pan cartridge S (vertical) → yoke →
  tilt cartridge S → head shell with a dark visor, eye lights and the RGB-D camera.
"""
from __future__ import annotations

import math

from build123d import Align, Axis, Box, Cylinder, Part, Pos, Rot, fillet

from ..params import BASELINE
from . import cartridge_m as cm, cartridge_s as cs
from .belt_stage import pulley
from .soft import soft_box, soft_shell

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
D = BASELINE
MAST_X = -60.0
W = D.upper.mast_w * 1000
Z_BASE = (D.base.clearance + D.base.height) * 1000
Z_TOP = Z_BASE + D.upper.spine_len * 1000
HEAD_Z = Z_TOP + 10.0
WALL = 2.0


def _rbox(x, y, z, r):
    r = min(r, x / 2 - 0.3, y / 2 - 0.3)
    return fillet(Box(x, y, z).edges().filter_by(Axis.Z), r) if r > 0.5 else Box(x, y, z)


def _shell(x, y, z, r, t=WALL):
    return _rbox(x, y, z, r) - _rbox(x - 2 * t, y - 2 * t, z - 2 * t, max(r - t, 1))


def mast() -> Part:
    L = Z_TOP - Z_BASE
    return Pos(MAST_X, 0, Z_BASE) * (Box(W, W, L, align=MIN) - Box(W - 4, W - 4, L, align=MIN))


def ratchet_strip() -> Part:
    """Printed in 4 segments (each ≤ 256 mm) bonded to the mast's back face."""
    z0, z1 = Z_BASE + 170, Z_TOP - 60
    seg = (z1 - z0) / 4
    p = None
    for i in range(4):
        s = Pos(MAST_X - W / 2 - 3, 23, z0 + i * seg) * Box(6, 14, seg - 1, align=MIN)
        for k in range(int(seg // 20)):                                  # visual teeth every 20 mm (real pitch 5 mm)
            s -= Pos(MAST_X - W / 2 - 6, 23, z0 + i * seg + 10 + 20 * k) * Rot(0, 45, 0) * Box(4, 16, 4)
        p = s if p is None else p + s
    return p


def lift_drive():
    z = Z_BASE + 40 + 70                                              # mount sits on top of the base coupling block
    T = Pos(MAST_X - W / 2 - 50, -20, z) * Rot(90, 0, 0)          # cartridge axis along -Y, behind the mast
    zc = cm.stack_height()
    return [("lift_can", T * Pos(0, 0, 0) * Rot(180, 0, 0) * Pos(0, 0, -zc) * cm.can(), "white"),
            ("lift_motor", T * Pos(0, 0, 0) * Rot(180, 0, 0) * Pos(0, 0, -zc) * cm.motor_dummy(), "dark"),
            ("lift_pulley", Pos(MAST_X - W / 2 - 50, -10, z) * Rot(-90, 0, 0) * pulley(20, 16), "orange"),
            ("lift_belt", Pos(MAST_X - W / 2 - 50 + 16, -2, z) * Box(3.6, 15, Z_TOP - z - 10, align=MIN), "dark"),
            ("lift_mount", lift_mount(z, zc), "white")]


def lift_mount(z, zc):
    """Mount block at the mast foot + the lift cartridge's torque fork (loop 20). The Ø6 torque pin sits 30 mm from the
    cartridge axis toward the mast (+X) and sticks out of the can rear (−Y). A 7 mm fork plate on an arm from the block's
    −Y face catches it. The slot is open toward the cartridge axis (radial float); torque load is vertical (±Z)."""
    xa = MAST_X - W / 2 - 50                                           # cartridge axis x
    m = Pos(MAST_X - W / 2 - 30, -20, Z_BASE + 40) * Box(60, 90, 120, align=MIN)
    m -= Pos(xa, -20, z) * Rot(90, 0, 0) * Cylinder(45, 200)
    tp = cm.TORQUE_PIN
    xp, y_out = xa + tp["r"], -20 - zc                                  # pin x; can rear face (y)
    xr = MAST_X - W / 2                                                # mast face
    m += Pos((xp + 14.3 + xr) / 2, (-65 + y_out - 9) / 2, z) * Box(xr - xp - 14.3, -65 - (y_out - 9), 48)      # arm beside the can (can r 43.3)
    m += Pos((xp - 7 + xr) / 2, y_out - 5.5, z) * Box(xr - xp + 7, 7, 48)                                       # fork plate
    m -= Pos(xp + 1.5, y_out - 5.5, z) * Box(17, 8, tp["d"] + 0.4)                                              # open radial slot
    m -= Pos(xa + 16, -2, z) * Box(3.6 + 6, 15 + 6, 400, align=MIN)        # R15: belt ≥ 3 mm from static parts                                            # loop 31: belt slot (the belt ran through the block)
    return m


def lift_cover() -> Part:
    """Looks pass v2: rounded cover over the lift cartridge, pulley and mount at the mast foot (slot for the belt)."""
    x0, x1, y0, y1, z0, z1 = -194.0, -91.0, -91.0, 36.0, Z_BASE + 41, Z_BASE + 172      # y0 −88 → −91: clears the torque-pin tip (loop 20)
    c = Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * soft_shell(x1 - x0, y1 - y0, z1 - z0, 30, 22, 2.0)
    c -= Pos((x0 + x1) / 2, (y0 + y1) / 2, z0) * Box(x1 - x0 + 2, y1 - y0 + 2, 6)      # open bottom
    c -= Pos(MAST_X - W / 2 - 34, -2, z1) * Box(26, 22, 10)                           # belt slot
    c -= Pos(x1, (y0 + y1) / 2, (z0 + z1) / 2) * Box(36, y1 - y0 + 4, z1 - z0 + 4)       # open against the mast / mount face
    zl = Z_BASE + 40 + 70
    c -= lift_mount(zl, cm.stack_height())                                            # loop 31: no overlap with the mount it covers
    return c


def carriage(lift_mm: float) -> Part:
    h = 160.0
    c = Pos(MAST_X, 0, lift_mm - h / 2) * (Box(W + 26, W + 26, h, align=MIN) - Box(W + 2, W + 2, h, align=MIN))
    for z in (lift_mm - h / 2 + 18, lift_mm + h / 2 - 18):           # 625 bearing bosses, 2 per face
        for dx, dy in ((W / 2 + 7, 0), (-W / 2 - 7, 0), (0, W / 2 + 7), (0, -W / 2 - 7)):
            c += Pos(MAST_X + dx, dy, z) * Box(14 if dx else 30, 30 if dx else 14, 24)
    c -= Pos(MAST_X - W / 2 - 6, 23, lift_mm) * Box(14, 18, 400)                 # slot over the ratchet strip
    # pawl + solenoid on the back, beside the slot
    c += Pos(MAST_X - W / 2 - 20, 23, lift_mm - 30) * Box(14, 22, 40)
    return c


def torso(lift_mm: float):
    # Looks pass (owner request): a rounded body that encloses both shoulder pitch modules; the arms emerge through
    # round ports at the shoulders. Printed as left/right halves (each 185 mm wide).
    x0 = 15.0
    shell = Pos(x0, 0, lift_mm) * soft_shell(230, 390, 210, 70, 40, 2.0)            # clears the pitch-bracket corners
    shell -= Pos(x0, 0, lift_mm) * Box(400, 1.0, 400)                                 # split line
    shell -= Pos(MAST_X, 0, lift_mm) * Box(W + 30, W + 30, 400)                       # mast + carriage passage
    for s in (1, -1):                                                                  # shoulder ports (pitch shaft + V-band)
        shell -= Pos(D.arm.shoulder_x * 1000, s * 195, lift_mm) * Rot(90, 0, 0) * Cylinder(50, 40)
    # electronics in front of the pitch cartridges (they reach into the torso at x −53 … 33)
    tray = Pos(75, 0, lift_mm - 40) * Box(60, 150, 6)
    pi = Pos(75, -20, lift_mm - 25.5) * Box(56, 85, 22)
    dcdc = Pos(75, 50, lift_mm - 23.5) * Box(50, 46, 26)
    ai_bay = Pos(75, 0, lift_mm + 25) * (Box(105, 95, 40) - Box(101, 91, 40))          # D10: reserved Jetson-class AI computer bay
    estop = Pos(-30, 80, lift_mm + 105) * Cylinder(15, 14, align=MIN)
    light = Pos(x0 + 116.5, 0, lift_mm - 30) * soft_box(3, 160, 8, 1.2, 0.6)
    return [("torso_shell", shell, "white"), ("ai_bay", ai_bay, "dark"), ("e_tray", tray, "dark"), ("e_pi5", pi, "dark"), ("e_dcdc", dcdc, "dark"),
            ("estop", estop, "orange"), ("chest_light", light, "orange")]


def head(pan_deg: float = 0.0, tilt_deg: float = 0.0):
    top = Pos(MAST_X, 0, Z_TOP) * Box(W + 16, W + 16, 24, align=MIN)                  # top bracket on the mast
    top -= Pos(MAST_X, 0, Z_TOP) * Box(W + 0.4, W + 0.4, 10, align=MIN)
    handle = Pos(MAST_X - 50, 0, Z_TOP + 24) * Rot(90, 0, 0) * (Cylinder(34, 24) - Cylinder(24, 25) - Pos(0, -40, 0) * Box(80, 80, 30))
    idler = Pos(MAST_X - W / 2 - 34, 0, Z_TOP + 8) * Rot(90, 0, 0) * Cylinder(14, 18)
    # pan cartridge S, shaft up, on the top bracket; yoke + head ride on it
    pan = Pos(MAST_X, 0, Z_TOP + 24) * Rot(0, 0, 90) * cs.sleeve()
    yz = Z_TOP + 24 + cs.stack_height()
    P = Pos(MAST_X, 0, yz) * Rot(0, 0, pan_deg)
    yoke = Box(60, 150, 8, align=MIN) + Pos(0, 71, 0) * Box(40, 8, 60, align=MIN) + Pos(0, -71, 0) * Box(40, 8, 60, align=MIN)
    tilt = Pos(0, 75, 40) * Rot(-90, 0, 0) * cs.sleeve()
    # loop 25: slotted torque pads for the pan and tilt cartridges' Ø4 pins (rnd/parts/torque.py)
    from .torque import with_pin_slot
    top = with_pin_slot(pan, top, 1.95, 2.0)
    yoke = with_pin_slot(tilt, yoke, 1.95, 2.0)
    T = Pos(0, 0, 40) * Rot(0, tilt_deg, 0)
    # D11 (loop 28): face screen. The shell grows 16 mm upward (120 → 136) so the 4.3" display board sits behind a flat
    # dark bezel with the depth camera below it.
    # D22 (owner, 2026-11-27): 7" Waveshare ESP32-P4-WIFI6-Touch-LCD-7B (1024 × 600, CAN). Head 244 × 136 → 252 × 184 mm.
    shell = Pos(70, 0, 28) * soft_shell(160, 252, 184, 58, 40, 2.4)                 # pill-shaped head
    shell -= Pos(150, 0, 40) * Box(20, 236, 124)                                     # bezel opening
    shell -= Pos(135, 0, -36) * Box(40, 96, 29)                                       # depth-camera window (Gemini 2)
    shell -= Pos(-20, 0, -20) * Box(110, 180, 90)                                    # neck opening at the back/bottom (yoke + pan servo)
    # bezel (dark PETG) with the screen window; Waveshare ESP32-S3-Touch-LCD-4.3B (112.4 × 75.1 mm board, 4.3" 800×480
    # capacitive touch, CAN transceiver, 7–36 V input: runs straight off the 24 V bus) behind it
    # loop 28: clearance for the tilt and pan cartridges over the whole tilt range (±30°) and any pan angle. The shell
    # used to clip the tilt sleeve (0.4 cm³) and, at +30° tilt, the pan sleeve.
    tb = tilt.bounding_box()
    env_t = Pos(tb.center().X, tb.center().Y, tb.center().Z) * Box(tb.size.X + 4, tb.size.Y + 4, tb.size.Z + 4)
    pb = (Pos(-MAST_X, 0, -yz) * pan).bounding_box()                                 # pan sleeve in the yoke (P) frame
    r_pan = max(math.hypot(x, y) for x in (pb.min.X, pb.max.X) for y in (pb.min.Y, pb.max.Y)) + 2
    env_p = Pos(0, 0, pb.min.Z - 2) * Cylinder(r_pan, pb.size.Z + 4, align=(Align.CENTER, Align.CENTER, Align.MIN))
    for td in range(-30, 31, 10):
        inv = (Pos(0, 0, 40) * Rot(0, td, 0)).inverse()
        shell -= inv * env_t
        shell -= inv * env_p
    visor = Pos(146, 0, 40) * soft_box(6, 232, 120, 2.8, 1.5) - Pos(146, 0, 40) * Box(10, 156, 90)   # 7" active area ≈ 154 × 86
    screen = Pos(138, 0, 40) * Box(9, 182, 112)                                       # board outline ESTIMATE (confirm with the drawing)
    eyes = [Pos(142.8, s * 34, 46) * Rot(0, 90, 0) * Cylinder(14, 0.6) for s in (1, -1)]      # drawn on the screen (render only)
    cam = Pos(128, 0, -36) * Box(30, 90, 25)                                         # Gemini 2, below the bezel
    out = [("head_bracket", top, "white"), ("handle", handle, "dark"), ("belt_idler", idler, "steel"),
           ("pan_sleeve", pan, "white"), ("yoke", P * yoke, "dark"), ("tilt_sleeve", P * tilt, "white"),
           ("head_shell", P * T * shell, "white"), ("visor", P * T * visor, "dark"),
           ("head_screen", P * T * screen, "black"),
           ("eye_l", P * T * eyes[0], "orange"), ("eye_r", P * T * eyes[1], "orange"), ("head_cam", P * T * cam, "dark")]
    return out


def module(lift_m: float = 0.75):
    L = lift_m * 1000
    return [("mast", mast(), "black"), ("ratchet", ratchet_strip(), "dark"), *lift_drive(), ("lift_cover", lift_cover(), "white"),
            ("carriage", carriage(L), "white"), *torso(L), *head()]


def masses(density=1.27):
    L = 750.0
    tor = {n: p for n, p, c in torso(L)}
    hd = {n: p for n, p, c in head()}
    return {
        "carriage (printed) + 8×625 bearings + pawl/solenoid": carriage(L).volume / 1e6 * density * 0.6 + 8 * 0.012 + 0.06,
        "torso shell + tray (printed)": (tor["torso_shell"].volume + tor["e_tray"].volume) / 1e6 * density,
        "ratchet strip (printed)": ratchet_strip().volume / 1e6 * density,
        "lift mount (printed) + belt 2.4 m + idler": [p for n, p, c in lift_drive() if n == "lift_mount"][0].volume / 1e6 * density * 0.5 + 0.10,
        "head: bracket, handle, yoke, shell, visor (printed)": sum(hd[k].volume for k in ("head_bracket", "handle", "yoke", "head_shell", "visor")) / 1e6 * density * 0.8,
        "head: 2 × STS3215 + sleeves + RGB-D camera (99 g) + eye LEDs": 2 * 0.055 + 2 * 0.02 + 0.10 + 0.02,
    }
