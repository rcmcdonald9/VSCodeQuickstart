"""Joint-side absolute encoders (Fable review M3, 2026-11-27): AS5048A magnet holders on the output side, PCB mounts on
the static side.  python -m rnd.parts.encoder   (writes docs/rnd/encoders.md)

sw/config/can_map.yaml expects an AS5048A on pitch, roll, yaw (shoulder node) and elbow (elbow node). The AS5048A is an
on-axis sensor: a diametrically magnetised magnet on the rotating shaft's axis, the IC centred over it on the static side.
Datasheet: ams AS5048A/AS5048B, 14-bit rotary position sensor, sections "Magnet requirements" and "Package drawings"
(the figures below are the ones this file designs to — re-check against the current revision before ordering):
  - magnet: diametric NdFeB cylinder, Ø6 × 2.5 mm used here (Ø8 × 3 also listed);
  - air gap, magnet face → IC package surface: 0.5 … 2.5 mm (designed to 1.0–1.3 mm here; where the IC reads through
    its board, the FR4 counts in the gap: it is non-magnetic);
  - package TSSOP-14: 5.0 × 6.4 mm body + leads, 1.2 mm max height (JEDEC MO-153);
  - lateral offset IC centre ↔ magnet axis: keep ≤ ±0.25 mm (VERIFY with the datasheet's error-vs-displacement plot):
    the PCB is located on the same printed part that holds the joint's bearing, never on a cover.
On-axis room, per joint (Fable M3: all four joints now have a mount):
  - elbow: the elbow tube's −Y end (the harness crosses on the +Y side) → bracket on the −Y 61808 boss;
  - pitch: the balancer crank's centre (inboard tube end) → a 0.8 mm FR4 strip cantilevered from a post on the inner wall
    (which holds pitch bearing A), crossing pin B's circle only where pin B never goes; the cable plane moved 2 mm inboard;
  - yaw: the yaw tube's top end (the ribbon leaves the bore lower down, at the clock-spring hub; C1's tie heads sit in the
    counterbore at r ≥ 8.25) → a strip board on pads on the top yaw plate (which holds yaw bearing A), IC hanging into the
    counterbore; the tube top is 0.5 mm under the plate top (roll.YAW_TOP);
  - inline roll: the roll clock-spring hub's +X end (a web in its bore carries the magnet; the ribbon's inner end enters the
    hub in front of it) → a board on a bridge across the static spring ring's cover hole.
  Steel fasteners near the magnets (pitch hub-tie studs r 15.5, yaw tie screws r 11, clevis M5 r 32) turn with the magnet,
  so they distort a field that rotates rigidly with it (offset / harmonics, calibrated at bring-up; VERIFY on the bench).
Frames: elbow parts in the upper-arm frame (rnd/parts/arm.py), pitch parts in the shoulder-module frame
(rnd/parts/shoulder.py), yaw and roll parts in the roll frame (rnd/parts/roll.py).
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
YAW_PAD = 1.0         # board on 1.0 mm pads on the top plate: 2.0 mm over the tube rim (swept target), 2.2 under the roll motor


def _yaw_geo():
    from . import roll
    z_plate = roll.P1[1]
    pcb0 = z_plate + YAW_PAD                             # board underside
    ic_bot = pcb0 - AS5048A["ic"][2]                     # IC on the underside, facing the magnet, hanging into the counterbore
    mag = ic_bot - 1.0                                   # magnet face: 1.0 mm air gap
    return dict(z_plate=z_plate, z_tube=roll.YAW_TOP, pcb=(pcb0, pcb0 + PCB_THIN), ic_bot=ic_bot, mag=mag, gap=ic_bot - mag,
                y=roll.ROLL_Y, screw_y=30.0, bore_r=roll.YF["bore_r"])


def yaw_magnet_holder() -> Part:
    """Printed plug pressed 8 mm into the yaw tube's Ø16 bore (C1: the ribbon bore), inside the tie ring (r 8.5) and the
    screw heads (r ≥ 8.25); the magnet flush in its top face, 0.2 mm under the tube top."""
    g = _yaw_geo()
    plug = Pos(0, g["y"], g["mag"] - 8.0) * Cylinder(g["bore_r"] - 0.05, 8.0, align=MIN)
    return plug - Pos(0, g["y"], g["mag"] - AS5048A["magnet"][1] + 0.01) * Cylinder(AS5048A["magnet"][0] / 2 + 0.05, AS5048A["magnet"][1], align=MIN)


def yaw_magnet() -> Part:
    g = _yaw_geo()
    return Pos(0, g["y"], g["mag"] - AS5048A["magnet"][1]) * Cylinder(AS5048A["magnet"][0] / 2, AS5048A["magnet"][1], align=MIN)


def yaw_pcb() -> Part:
    """Strip board 14 × 66 × 0.8 (ESTIMATE) across the tube end along Y (along X it would reach the −X clevis arm's foot),
    the IC on its underside over the axis, 2 × M2 at y ±30 into the top plate on 1.0 mm pads (modelled with the board)."""
    g = _yaw_geo()
    p0, p1 = g["pcb"]
    b = Pos(0, g["y"], p0) * Box(14, 66, p1 - p0, align=MIN)
    b += Pos(0, g["y"], g["ic_bot"]) * Box(AS5048A["ic"][0], AS5048A["ic"][1], AS5048A["ic"][2], align=MIN)
    for dy in (-g["screw_y"], g["screw_y"]):
        b += Pos(0, g["y"] + dy, g["z_plate"]) * (Cylinder(4.0, YAW_PAD, align=MIN) - Cylinder(1.1, YAW_PAD, align=MIN))      # pads
    return b


def yaw_pilot_cuts() -> Part:
    """2 × Ø1.8 × 6 pilots for the M2 board screws in the top yaw plate (roll.clevis_and_yaw_block)."""
    g = _yaw_geo()
    a, b = (Pos(0, g["y"] + dy, g["z_plate"] - 6.0) * Cylinder(0.9, 6.01, align=MIN) for dy in (-g["screw_y"], g["screw_y"]))
    return a + b


def yaw_parts():
    """(static board, rotating holder + magnet): roll.module_inline() adds the board as "yaw_enc_pcb" and unions the
    holder + magnet into "yaw_shaft" (they turn with it; rnd/swept.py)."""
    return [("enc_pcb", yaw_pcb(), "green")], [("enc_magnet_holder", yaw_magnet_holder(), "dark"), ("enc_magnet", yaw_magnet(), "steel")]


# ---------------- pitch: balancer crank centre (rotates with the arm) → strip on a post on the inner wall (static) ----------------
# Shoulder-module frame: pitch axis = Y, x forward, z up, inboard = −Y. Pin B (r 20 on the crank, standing into the cable
# plane) points along the hanging arm: at pitch θ it sits at azimuth −90° − θ (robot.build turns the pitch output by
# Rot(0, θ, 0)), so over twin.JR pitch −200…60° it never visits azimuths 110…210°. The strip crosses pin B's circle
# (r 17–23) at 133° (±17° with its 3 mm half-width and the pin's r 3: 116…150°). Its post sits outside r 26 (the 61808
# seat), between pulley A's lowest slot position (Ø28 tyre at a 38: 1.5 mm) and the path of the pitch cartridge's 40° ear
# when the cartridge leaves through its downward side exit (D20 ear r 56: x ≤ −21.6; 0.8 mm). Stack from the crank's
# inboard face (y −59.3): 0.5 running gap → FR4 0.8 → IC 1.2 on the inboard side, read through the board (magnet →
# package 1.3 mm) → 0.5 mm to the cable zone (Ø2 cable + 0.4, plane shoulder.Y_BAL −63.5, was −61.5).
PITCH_STRIP = dict(w=6.0, pad=9.0, cross=(23.5, 133.0), posts=((-17.8, 24.2), (-14.9, 27.0)), post_r=3.0, gap=0.5)


def _pitch_geo():
    import math
    from . import shoulder
    y_c = shoulder.Y_TUBE_IN - shoulder.CRANK_T                    # crank inboard face (magnet face)
    b0 = y_c - PITCH_STRIP["gap"]
    r, a = PITCH_STRIP["cross"]
    return dict(y_c=y_c, pcb=(b0 - PCB_THIN, b0), ic=(b0 - PCB_THIN - AS5048A["ic"][2], b0 - PCB_THIN), gap=PITCH_STRIP["gap"] + PCB_THIN,
                wall=shoulder.Y_F, B=(r * math.cos(math.radians(a)), r * math.sin(math.radians(a))), y_bal=shoulder.Y_BAL)


def _seg(p, q, w, y0, t) -> Part:
    """Bar of width w from p to q (module x, z) with round ends, y0 … y0 + t."""
    import math
    dx, dz = q[0] - p[0], q[1] - p[1]
    bar = Pos((p[0] + q[0]) / 2, y0, (p[1] + q[1]) / 2) * Rot(0, -math.degrees(math.atan2(dz, dx)), 0) * Box(math.hypot(dx, dz), t, w, align=(Align.CENTER, Align.MIN, Align.CENTER))
    for c in (p, q):
        bar += Pos(c[0], y0, c[1]) * Rot(-90, 0, 0) * Cylinder(w / 2, t, align=MIN)
    return bar


def pitch_magnet_pocket() -> Part:
    """Ø6.1 × 2.5 pocket in the crank's inboard-face centre (7075: 1.5 mm left; magnet bonded in, face flush)."""
    g = _pitch_geo()
    return Pos(0, g["y_c"] - 0.01, 0) * Rot(-90, 0, 0) * Cylinder(AS5048A["magnet"][0] / 2 + 0.05, AS5048A["magnet"][1] + 0.01, align=MIN)


def pitch_magnet() -> Part:
    g = _pitch_geo()
    return Pos(0, g["y_c"], 0) * Rot(-90, 0, 0) * Cylinder(AS5048A["magnet"][0] / 2, AS5048A["magnet"][1], align=MIN)


def pitch_pcb() -> Part:
    """FR4 0.8 strip (ESTIMATE layout: 6 mm wide, 6 SPI + power traces): a 9 × 9 pad on the axis with the AS5048A on its
    inboard face, a leg along 133° to r 23.5, a dog-leg to the two M2 holes on the post."""
    g = _pitch_geo()
    y0, y1 = g["pcb"]
    w, (q1, q2) = PITCH_STRIP["w"], PITCH_STRIP["posts"]
    b = Pos(0, y0, 0) * Box(PITCH_STRIP["pad"], y1 - y0, PITCH_STRIP["pad"], align=(Align.CENTER, Align.MIN, Align.CENTER))
    b += _seg((0.0, 0.0), g["B"], w, y0, y1 - y0) + _seg(g["B"], q1, w, y0, y1 - y0) + _seg(q1, q2, w, y0, y1 - y0)
    for q in (q1, q2):
        b -= Pos(q[0], y0 - 1, q[1]) * Rot(-90, 0, 0) * Cylinder(1.1, 3, align=MIN)
    i0, i1 = g["ic"]
    return b + Pos(0, i0, 0) * Box(AS5048A["ic"][0], i1 - i0, AS5048A["ic"][1], align=(Align.CENTER, Align.MIN, Align.CENTER))


def pitch_post() -> Part:
    """Post on the inner wall's inboard face (part of shoulder.bracket_body, the print that holds pitch bearing A): a 6 mm
    stadium from the wall to the board, 2 × M2 pilots (Ø1.6 × 5)."""
    g = _pitch_geo()
    q1, q2 = PITCH_STRIP["posts"]
    y0, y1 = g["pcb"][1], g["wall"]
    p = _seg(q1, q2, 2 * PITCH_STRIP["post_r"], y0, y1 - y0 + 0.5)
    for q in (q1, q2):
        p -= Pos(q[0], y0 - 0.01, q[1]) * Rot(-90, 0, 0) * Cylinder(0.8, 5.0, align=MIN)
    return p


def pitch_parts():
    """Static board only: shoulder.module() adds it as "enc_pcb"; the magnet is unioned into the crank (pitch_hub_crank)."""
    return [("enc_pcb", pitch_pcb(), "green")]


# ---------------- inline roll: spring-hub web (rotates with the roll output) → board on the spring ring's cover (static) ----------------
# Roll frame, roll axis = X through (y ROLL_Y, z 0). The hub sleeve (r 20.1–22, x 62 … 72.8) gets a 3.5 mm web at
# x 66 … 69.5, the magnet flush in its +X face; the static spring ring's cover (x 73.3 … 74.9, Ø45 hole) gets a 12 mm bridge
# across the hole carrying a 14 × 14 × 1.6 board on its −X face (inside the hub bore, 10 mm radial clearance), the IC
# facing the magnet: 1.0 mm air gap. The ribbon's inner end enters the hub through its sleeve at x 62 … 66 (in front of
# the web) and turns into the clevis arm's Ø16 bore. The ring is located on the roll link's spring bar (VERIFY the ±0.25
# lateral offset; else calibrate the offset at bring-up).
ROLL_ENC = dict(web=(66.0, 69.5), bridge_w=12.0, pcb=14.0, pcb_t=1.6, screw_z=5.0)


def _roll_geo():
    from . import roll
    x1 = roll.ROLL_SPR_X[1]
    p1 = x1                                           # cover x1 … x1 + 1.6 (roll.roll_spring_ring); board on its −X face
    p0 = p1 - ROLL_ENC["pcb_t"]
    ic0 = p0 - AS5048A["ic"][2]
    return dict(y=roll.ROLL_Y, web=ROLL_ENC["web"], mag=ROLL_ENC["web"][1], pcb=(p0, p1), ic=(ic0, p0), gap=ic0 - ROLL_ENC["web"][1],
                cover=(x1, x1 + 1.6), hole_r=roll.ROLL_SPR[0] + 0.5)


def roll_hub_web() -> Part:
    """Added to roll.roll_spring_hub(): the web (r 20.3, fused into the r 20.1 sleeve bore) minus the magnet pocket."""
    g = _roll_geo()
    w0, w1 = g["web"]
    web = Pos(w0, g["y"], 0) * Rot(0, 90, 0) * Cylinder(20.3, w1 - w0, align=MIN)
    return web - Pos(w1 - AS5048A["magnet"][1], g["y"], 0) * Rot(0, 90, 0) * Cylinder(AS5048A["magnet"][0] / 2 + 0.05, AS5048A["magnet"][1] + 0.01, align=MIN)


def roll_magnet() -> Part:
    g = _roll_geo()
    return Pos(g["mag"] - AS5048A["magnet"][1], g["y"], 0) * Rot(0, 90, 0) * Cylinder(AS5048A["magnet"][0] / 2, AS5048A["magnet"][1], align=MIN)


def roll_bridge() -> Part:
    """Added to roll.roll_spring_ring(): a bridge across the cover hole in the cover plane (into the cover past r 22.5),
    2 × M2 pilots (Ø1.6) at z ±5 for the board."""
    g = _roll_geo()
    c0, c1 = g["cover"]
    br = Pos(c0, g["y"], 0) * Box(c1 - c0, 2 * g["hole_r"] + 4.0, ROLL_ENC["bridge_w"], align=(Align.MIN, Align.CENTER, Align.CENTER))
    for z in (-ROLL_ENC["screw_z"], ROLL_ENC["screw_z"]):
        br -= Pos(c0 - 0.01, g["y"], z) * Rot(0, 90, 0) * Cylinder(0.8, c1 - c0 + 0.02, align=MIN)
    return br


def roll_pcb() -> Part:
    g = _roll_geo()
    p0, p1 = g["pcb"]
    b = Pos(p0, g["y"], 0) * Box(p1 - p0, ROLL_ENC["pcb"], ROLL_ENC["pcb"], align=(Align.MIN, Align.CENTER, Align.CENTER))
    for z in (-ROLL_ENC["screw_z"], ROLL_ENC["screw_z"]):
        b -= Pos(p0 - 0.01, g["y"], z) * Rot(0, 90, 0) * Cylinder(1.1, p1 - p0 + 0.02, align=MIN)
    i0, i1 = g["ic"]
    return b + Pos(i0, g["y"], 0) * Box(i1 - i0, AS5048A["ic"][1], AS5048A["ic"][0], align=(Align.MIN, Align.CENTER, Align.CENTER))


def roll_parts():
    """(static board, rotating magnet): roll.module_inline() adds the board as "roll_enc_pcb" and unions the magnet into
    "roll_spring_hub" (on the roll output)."""
    return [("enc_pcb", roll_pcb(), "green")], [("enc_magnet", roll_magnet(), "steel")]


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
    # yaw, inline roll (built into roll.module_inline) and pitch (shoulder.module): each board against its module
    def _hits(board, mod, skip):
        out = []
        for n, q in mod.items():
            if n in skip or "enc_pcb" in n:
                continue
            v = _clear(board, q)
            if v > 0.05:
                out.append((n, round(v, 2)))
        return out

    def _ok(gap):
        return "✅" if AS5048A["gap"][0] <= gap <= AS5048A["gap"][1] else "❌"

    mod = {n: p for n, p, _ in roll.module()}
    y = _yaw_geo()
    L += ["", "## Yaw (built: rnd/parts/roll.py module_inline, `yaw_enc_pcb`; holder + magnet in `yaw_shaft`)", "",
          f"- Magnet in a plug in the yaw tube's Ø16 bore (inside C1's tie ring), face at z {y['mag']:.1f}; tube top trimmed to z {y['z_tube']:.1f} "
          f"(roll.YAW_TOP, 1.0 under the plate; the tie screws M3 × {roll.YAW_TIE['L']:.0f}, {roll.yaw_tie_levels()['engaged']:.1f} mm engaged). Board "
          f"14 × 66 × 0.8 on 1.0 mm pads on the top yaw plate (2 × M2 at y ROLL_Y ±{y['screw_y']:.0f}), IC on its underside: air gap {y['gap']:.1f} mm "
          f"{_ok(y['gap'])}; board top z {y['pcb'][1]:.1f} under the roll motor (r {roll.R_MOT:.0f} → z −{roll.R_MOT:.0f} on the yaw axis).",
          f"- Overlaps with the roll module: {_hits(mod.get('yaw_enc_pcb', yaw_pcb()), mod, ('yaw_block',)) or 'none'} (the pads sit on the yaw block)."]
    if roll.INLINE:
        r = _roll_geo()
        L += ["", "## Shoulder roll, inline (built: roll.module_inline, `roll_enc_pcb`; web + magnet in `roll_spring_hub`)", "",
              f"- Magnet in a web inside the spring hub (x {r['web'][0]:.1f}–{r['web'][1]:.1f}), board 14 × 14 × 1.6 on a bridge across the spring ring's "
              f"cover hole, IC facing the magnet: air gap {r['gap']:.1f} mm {_ok(r['gap'])}.",
              f"- Overlaps with the roll module: {_hits(mod.get('roll_enc_pcb', roll_pcb()), mod, ('roll_spring_ring',)) or 'none'} (it sits on the bridge)."]
    from . import shoulder
    pg = _pitch_geo()
    smod = {n: p for n, p, _ in shoulder.module()}
    L += ["", "## Shoulder pitch (built: rnd/parts/shoulder.py module, `enc_pcb`; magnet in `pitch_hub_crank`, post on `bracket`)", "",
          f"- Magnet flush in the crank's inboard face (y {pg['y_c']:.1f}), FR4 0.8 strip on a post on the inner wall, IC on its inboard face read "
          f"through the board: magnet → package {pg['gap']:.1f} mm {_ok(pg['gap'])}; IC to the cable zone (plane {pg['y_bal']:.1f}, Ø2 + 0.4): "
          f"{pg['ic'][0] - (pg['y_bal'] + 1.2):.1f} mm.",
          f"- Overlaps with the shoulder module: {_hits(smod.get('enc_pcb', pitch_pcb()), smod, ('bracket',)) or 'none'} (it sits on the post); "
          "pin B never reaches the strip (azimuths 110…210° unvisited); the cable check is rnd/balancer.py (cad_plane_clear)."]
    (DOCS / "encoders.md").write_text("\n".join(L))
    print("\n".join(L))
    return clashes


if __name__ == "__main__":
    report()
