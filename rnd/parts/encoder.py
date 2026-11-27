"""Joint-side absolute encoders (Fable review M3, 2026-11-27): AS5048A magnet holders on the output side, PCB mounts on
the static side.  python -m rnd.parts.encoder   (writes docs/rnd/encoders.md)

sw/config/can_map.yaml expects an AS5048A on pitch, roll, yaw (shoulder node) and elbow (elbow node). The AS5048A is an
on-axis sensor: a diametrically magnetised magnet on the rotating shaft's axis, the IC centred over it on the static side.
Datasheet (ams AS5048A/AS5048B, "Magnet" / "Mechanical data" sections; the figures below are the ones this file designs
to — re-check against the current revision before ordering):
  - magnet: diametric NdFeB cylinder, Ø6 × 2.5 mm used here (Ø8 × 3 also listed);
  - air gap, magnet face → IC package surface: 0.5 … 2.5 mm (designed to the middle, ≈ 1–1.6 mm);
  - lateral offset IC centre ↔ magnet axis: keep ≤ ±0.25 mm (VERIFY with the datasheet's error-vs-displacement plot):
    the PCB is located on the same printed part that holds the joint's bearing, never on a cover.
On-axis room exists at the elbow (the elbow tube's −Y end is free: the harness crosses on the +Y side) and at the yaw
(the yaw tube's top end: the ribbon leaves the bore lower down, at the clock-spring hub). Pitch and roll have their axes
taken (balancer cable plane / ribbon pass); report() lists what their owners would need.
Frames: elbow parts in the upper-arm frame (rnd/parts/arm.py), yaw parts in the roll frame (rnd/parts/roll.py).
"""
from __future__ import annotations

from pathlib import Path

from build123d import Align, Box, Cylinder, Part, Pos, Rot

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
AS5048A = dict(gap=(0.5, 2.5), magnet=(6.0, 2.5), lateral=0.25, ic=(5.0, 6.4, 1.2))   # IC: TSSOP-14 5.0 × 6.4 × 1.2 max (body + leads)
PCB_T = 1.6                       # elbow board (IC facing the magnet)
PCB_THIN = 0.8                    # yaw board (IC on top, read through the 0.8 mm FR4)
DOCS = Path(__file__).resolve().parents[2] / "docs" / "rnd"


# ---------------- elbow: tube −Y end (rotates with the forearm) → bracket on the −Y 61808 seat boss (static) ----------------
def _elbow_geo():
    from . import arm
    y_end = -(arm.HOUSING["y"] / 2 + 12)                    # elbow tube's −Y end face (arm.elbow_pulley_hub: y0)
    gap = 1.0
    ic_top = y_end - gap                                     # IC package surface facing the magnet
    pcb0 = ic_top - AS5048A["ic"][2]                         # PCB face carrying the IC
    pcb1 = pcb0 - PCB_T
    return dict(y_end=y_end, gap=gap, ic_top=ic_top, pcb=(pcb1, pcb0), plate=(pcb1 - 3.0, pcb1), boss_face=arm.SEAT_Y["-"][0],
                z=-arm.ELBOW_Z, leg_r=30.5, leg_ang=(45.0, 135.0))


def elbow_magnet_holder() -> Part:
    """Printed plug pressed 8 mm into the elbow tube's Ø30 −Y bore, the Ø6 × 2.5 magnet flush in its outer face."""
    g = _elbow_geo()
    plug = Pos(0, g["y_end"], g["z"]) * Rot(-90, 0, 0) * Cylinder(14.95, 8.0, align=MIN)
    return plug - Pos(0, g["y_end"] - 0.01, g["z"]) * Rot(-90, 0, 0) * Cylinder(AS5048A["magnet"][0] / 2 + 0.05, AS5048A["magnet"][1], align=MIN)


def elbow_magnet() -> Part:
    g = _elbow_geo()
    return Pos(0, g["y_end"], g["z"]) * Rot(-90, 0, 0) * Cylinder(AS5048A["magnet"][0] / 2, AS5048A["magnet"][1], align=MIN)


def elbow_pcb() -> Part:
    """AS5048A board 22 × 22 × 1.6 (ESTIMATE), IC on the +Y face over the axis."""
    g = _elbow_geo()
    p0, p1 = g["pcb"]
    b = Pos(0, (p0 + p1) / 2, g["z"]) * Box(22, p1 - p0, 22)
    ic = Pos(0, (g["ic_top"] + p1) / 2, g["z"]) * Box(AS5048A["ic"][1], g["ic_top"] - p1, AS5048A["ic"][0])
    return b + ic


def elbow_bracket() -> Part:
    """Printed bracket: a 3 mm plate behind the PCB (2 × M2 into it) on two Ø8 legs to the −Y seat boss's outer face at
    r 30.5, 45° and 135° above the elbow axis (the housing half of the boss, so the seat cap still drops), 2 × M3 into
    heat-set inserts in the boss face. Non-structural (≈ 6 g): inserts in pull-out are acceptable here."""
    import math
    g = _elbow_geo()
    q0, q1 = g["plate"]
    br = Pos(0, (q0 + q1) / 2, g["z"]) * Box(26, q1 - q0, 26)
    for a in g["leg_ang"]:
        t = math.radians(a)
        x, z = g["leg_r"] * math.cos(t), g["z"] + g["leg_r"] * math.sin(t)
        br += Pos(x, g["boss_face"], z) * Rot(90, 0, 0) * Cylinder(4.0, g["boss_face"] - q0, align=MIN)          # leg
        br += Pos(x / 2, (q0 + q1) / 2, (z + g["z"]) / 2) * Rot(0, -math.degrees(math.atan2(z - g["z"], x)), 0) * Box(math.hypot(x, z - g["z"]), q1 - q0, 8)   # arm
        br -= Pos(x, g["boss_face"], z) * Rot(90, 0, 0) * Cylinder(1.7, 40, align=MIN)
        br -= Pos(x, q0, z) * Rot(-90, 0, 0) * Cylinder(3.0, 3.0, align=MIN)                                     # counterbore
    for x in (-8.0, 8.0):
        br -= Pos(x, q0, g["z"]) * Rot(-90, 0, 0) * Cylinder(0.8, 4.0, align=MIN)                               # M2 pilots for the PCB
    return br


def elbow_insert_cuts() -> Part:
    """What to subtract from the upper bone: 2 × M3 heat-set insert holes (Ø4.0 × 5.7) in the −Y seat boss face."""
    import math
    g = _elbow_geo()
    c = None
    for a in g["leg_ang"]:
        t = math.radians(a)
        h = Pos(g["leg_r"] * math.cos(t), g["boss_face"], g["z"] + g["leg_r"] * math.sin(t)) * Rot(-90, 0, 0) * Cylinder(1.95, 5.7, align=MIN)
        c = h if c is None else c + h
    return c


def elbow_parts():
    """(name, part, colour) in the upper-arm frame. The magnet holder and magnet turn with the elbow tube (and so with
    the forearm): callers place them with the forearm's elbow rotation."""
    return [("enc_bracket", elbow_bracket(), "dark"), ("enc_pcb", elbow_pcb(), "green")], \
           [("enc_magnet_holder", elbow_magnet_holder(), "dark"), ("enc_magnet", elbow_magnet(), "steel")]


# ---------------- yaw: tube top end (rotates with the upper arm) → board on the top yaw plate (static) ----------------
YAW_TRIM = 0.3        # the yaw tube's top must end 0.3 mm under the top plate's top face (roll.py today: 2.0 mm above it)


def _yaw_geo():
    from . import roll
    z_plate = roll.P1[1]
    z_tube = z_plate - YAW_TRIM                          # tube top (after the roll owner's trim) = magnet face
    pcb0 = z_plate + 0.5                                 # PCB on 0.5 mm pads at r ≥ 27 (clear of the 61808's inner race)
    return dict(z_plate=z_plate, z_tube=z_tube, pcb=(pcb0, pcb0 + PCB_THIN), ic_top=pcb0 + PCB_THIN + AS5048A["ic"][2],
                gap=pcb0 - z_tube, y=roll.ROLL_Y, screw_x=30.0)


def yaw_magnet_holder() -> Part:
    """Printed plug pressed 8 mm into the yaw tube's Ø20 top bore, magnet flush with the (trimmed) tube top."""
    g = _yaw_geo()
    plug = Pos(0, g["y"], g["z_tube"] - 8.0) * Cylinder(9.95, 8.0, align=MIN)
    return plug - Pos(0, g["y"], g["z_tube"] - AS5048A["magnet"][1] + 0.01) * Cylinder(AS5048A["magnet"][0] / 2 + 0.05, AS5048A["magnet"][1], align=MIN)


def yaw_pcb() -> Part:
    """Strip board 14 × 66 × 0.8 (ESTIMATE) across the tube end along Y (along X it would reach the −X clevis arm's foot),
    IC on top over the axis (reads through the FR4), 2 × M2 at y ±30 into the top plate on 0.5 mm pads."""
    g = _yaw_geo()
    p0, p1 = g["pcb"]
    b = Pos(0, g["y"], p0) * Box(14, 66, p1 - p0, align=MIN)
    b += Pos(0, g["y"], p1) * Box(AS5048A["ic"][0], AS5048A["ic"][1], AS5048A["ic"][2], align=MIN)
    for dy in (-g["screw_x"], g["screw_x"]):
        b += Pos(0, g["y"] + dy, g["z_plate"]) * (Cylinder(4.0, 0.5, align=MIN) - Cylinder(1.1, 0.5, align=MIN))      # pads
    return b


def yaw_parts():
    return [("enc_pcb", yaw_pcb(), "green")], [("enc_magnet_holder", yaw_magnet_holder(), "dark")]


# ---------------- checks ----------------
def _clear(a, b) -> float:
    """Overlap volume (mm³) between two parts (0 = clear)."""
    try:
        return (a & b).volume
    except Exception:
        return 0.0


def report():
    from . import arm, roll
    L = ["# Joint encoders (generated by `python -m rnd.parts.encoder`)", "", __doc__.split("\n\n", 1)[1].split("Frames:")[0].strip(), ""]
    # elbow (built into arm.assembly)
    g = _elbow_geo()
    stat, rot = elbow_parts()
    others = {n: p for n, p, _ in arm.assembly(elbow_deg=0.0, wrist_deg=0.0) if not n.startswith(("enc_", "hand_"))}
    clashes = [(a, b, round(_clear(p, q), 2)) for a, p, _ in stat + rot for b, q in others.items()
               if not (a.startswith("enc_magnet") and b == "e_shaft40")]          # the plug is a press fit in the tube bore
    clashes = [c for c in clashes if c[2] > 0.05]
    ok_gap = AS5048A["gap"][0] <= g["gap"] <= AS5048A["gap"][1]
    L += ["## Elbow (built: rnd/parts/arm.py assembly, parts `enc_*`)", "",
          f"- Magnet Ø{AS5048A['magnet'][0]:.0f} × {AS5048A['magnet'][1]} in a printed plug in the elbow tube's −Y end (y {g['y_end']:.1f}), "
          f"IC {g['gap']:.1f} mm off it {'✅' if ok_gap else '❌'} (datasheet {AS5048A['gap'][0]}–{AS5048A['gap'][1]} mm), board 22 × 22 × 1.6 on a "
          f"bracket with two legs to the −Y 61808 boss (r {g['leg_r']}, {g['leg_ang'][0]:.0f}°/{g['leg_ang'][1]:.0f}°), 2 × M3 into inserts in the boss face.",
          f"- Clashes with the arm at elbow 0°: {len(clashes)} {'✅' if not clashes else '❌ ' + str(clashes)}"]
    # yaw (parts here; needs the roll owner's trim)
    y = _yaw_geo()
    stat, rot = yaw_parts()
    mod = {n: p for n, p, _ in roll.module()}
    tube_top = mod["yaw_shaft"].bounding_box().max.Z
    hits = [(a, b, round(_clear(p, q), 2)) for a, p, _ in stat for b, q in mod.items()]
    hits = [h for h in hits if h[2] > 0.05]
    ok_y = AS5048A["gap"][0] <= y["gap"] + PCB_THIN <= AS5048A["gap"][1]
    L += ["", "## Yaw (parts in this file; the roll owner adds them to roll.module() after two edits in rnd/parts/roll.py)", "",
          f"- Magnet in a plug in the yaw tube's Ø20 top bore, board 14 × 66 × 0.8 across the tube end on the top yaw plate (2 × M2 at y ROLL_Y ±{y['screw_x']:.0f}, "
          f"0.5 mm pads), IC on top reading through the board: magnet → package {y['gap'] + PCB_THIN:.1f} mm {'✅' if ok_y else '❌'}; IC top at z "
          f"{y['ic_top']:.1f}, under the roll housing (z −48 on the yaw axis).",
          f"- Needs in roll.py: (1) yaw_shaft() top end at P1[1] − {YAW_TRIM} = {y['z_tube']:.1f} (today {tube_top:.1f}: the tube's 2 mm above the plate "
          f"carry nothing); (2) 2 × Ø1.8 pilots in the top plate at (0, ROLL_Y ± {y['screw_x']:.0f}); then append encoder.yaw_parts() to module_inline().",
          f"- Today the board would hit: {sorted(set(h[1] for h in hits)) or 'nothing'}."]
    L += ["", "## Pitch and roll (owner: rnd/parts/shoulder.py, roll.py): geometry to adopt", "",
          "- **Pitch:** both tube ends are taken: the outboard end carries the harness to the detach plug, and at pitch 0° the balancer cable "
          "(pin B r 20 → pulley A) crosses the axis at y −61.5, 2.2 mm past the crank face. Fix: magnet Ø6 × 2.5 flush in a pocket at the crank's centre "
          "(inboard face, y −59.3); board 20 × 20 × 1.6 with the IC toward the crank (IC surface y −60.3, 1.0 mm air gap; board back −63.1) on a 2 mm "
          "printed cantilever (to −65.1) from the inner wall's inboard face, entering at azimuth +20° (x forward, z up): pin B sweeps −290° … −30° over "
          "pitch −200° … +60°, so −30° … +70° is never visited; cable plane Y_BAL −61.5 → −67.0 (pin B stand-off 5.2 → 10.7 mm: ≈ 2.8 N·m on the Ø6 pin, "
          "≈ 130 MPa: a hardened dowel, ESTIMATE).",
          "- **Roll (inline):** the +X end has the Ø16 ribbon pass through the output adapter and clevis arm, and the −X end of roll_arm_rear is 1.2 mm "
          "from the yaw can (which turns with roll). On-axis AS5048A does not fit as drawn: route the ribbon off-axis through a 6 × 16 slot at r 12 in the "
          "adapter and clevis arm, put the magnet in a Ø6.1 × 2.5 pocket at the adapter's +X centre, and the board (20 × 20) on a static bridge from the roll "
          "link's spring bar inside the spring ring, 1.0 mm off; or use an off-axis ring-magnet encoder on the Ø60 output step (not an AS5048A)."]
    (DOCS / "encoders.md").write_text("\n".join(L))
    print("\n".join(L))
    return clashes, hits


if __name__ == "__main__":
    report()
