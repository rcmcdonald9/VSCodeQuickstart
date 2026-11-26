"""Self-docking charger, printed parts (queue item 4, loop 26). Validated by rnd/dock_sim.py (100/100 docks).

Dock frame: floor at z = 0, dock axis +X (the robot arrives from +X), V apex at the origin.
- **Two wings**: 10 mm walls, 250 mm faces at ±60° to the axis (a round R 250 base centres itself in the V), 160 mm tall
  (the skirt spans 25–155 mm), with floor tabs for 2 screws each. Printed as mirror-image parts.
- **Contact tower** at the apex: reaches 39 mm toward the robot (the pads on the skirt sit 39 mm from the apex when the base
  is seated). It carries 2 spring leaf contacts (+ / −, at 60 and 100 mm) and a spring-loaded ID/handshake pin (at 40 mm).
  The charger output stays off until the robot answers on the ID pin, so no live contacts at floor level.
- A low-profile lidar target: the V's two faces give a distinctive 120° corner in the 2D scan. An AprilTag goes on the
  tower top for the head camera.
"""
from __future__ import annotations

import math

from build123d import Align, Box, Cylinder, Part, Pos, Rot

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
V_DEG, FACE_L, WALL_T, H = 60.0, 250.0, 10.0, 160.0     # 10 mm walls: the dock is screwed down, it needs no mass
R_BASE = 250.0
SEAT_X = R_BASE / math.sin(math.radians(V_DEG))           # base centre when seated (289 mm)
PAD_GAP = SEAT_X - R_BASE                                  # apex → pad (39 mm)


def wing(side: int = 1) -> Part:
    a = math.radians(V_DEG)
    w = Pos(FACE_L / 2, side * WALL_T / 2, 0) * Box(FACE_L, WALL_T, H, align=MIN)              # face along +X before rotation
    w += Pos(FACE_L - 20, side * (WALL_T + 15), 0) * (Box(40, 30, 5, align=MIN) - Pos(0, 0, 0) * Cylinder(2.4, 20))   # floor tab
    return Rot(0, 0, side * V_DEG) * w


def tower() -> Part:
    t = Pos(PAD_GAP / 2 - 10, 0, 0) * Box(PAD_GAP + 20, 50, H - 10, align=MIN)
    t -= Pos(PAD_GAP - 2, 0, 60) * Box(8, 12, 30)                                              # leaf-contact pockets (+ at 60, − at 100)
    t -= Pos(PAD_GAP - 2, 0, 100) * Box(8, 12, 30)
    t -= Pos(PAD_GAP - 6, 0, 40) * Rot(0, 90, 0) * Cylinder(3.2, 20)                           # ID/handshake pogo pin
    return t


def parts():
    return [("dock_wing_l", wing(1), "orange"), ("dock_wing_r", wing(-1), "orange"), ("dock_tower", tower(), "dark")]
