"""Assembly-sequence check (external review #1, finding 1 → queue A1).  python -m rnd.assembly_seq

The clash checks prove that parts don't overlap; they don't prove that the parts can be put together. This checks two
things for every drive shaft:

1. **Loose-part slide-on.** Each bearing (bore d) **and each separate pulley or hub** (its smallest bore: a hex or
   double-D counts by its across-flats) must reach its seat from at least one end of the *same printed piece* without
   passing a section wider than d/2 (radius profile sampled every 0.5 mm along the shaft axis). Astra review #2b found
   that loop 40's split pulleys failed this (a Ø40 round tube can't pass a 34.4 mm hex bore); the pulley rows were added.
2. **Shaft insertion.** A one-piece shaft carrying an element wider than the bearing seats of the walls it passes
   through (an integral pulley, flange or ferrule between or beyond two walls of a one-piece link) can't be put in.
   The shaft must be split (bolted pulley / flange), or the link must be split.
3. **Tool access (R14 (c), Fable review C3).** Every drive-train fastener (cartridge flange ears, rear caps, seat caps,
   pinch clamps, pitch hub, yaw flange, the inline roll's motor and clevis bolts) must be reachable on the assembled robot
   with a real driver: Ø6 straight for 80 mm, or a ball-end at ≤ 25°.
4. **Cartridge swap (R14 (d)).** Each bolted cartridge must come out with the belt on and the shaft in place.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from build123d import Box, Pos, Rot

DOCS = Path(__file__).resolve().parent.parent / "docs" / "rnd"


def profile(part, z0, z1, step=0.5):
    """Max radius about the local Z axis for slabs z ∈ [z0, z1] (part already in its axis frame)."""
    zs, rs = [], []
    for z in np.arange(z0, z1 + 1e-9, step):
        sl = part & Pos(0, 0, z) * Box(400, 400, step)
        try:
            v = sl.volume
        except Exception:
            v = 0
        if v <= 1e-6:
            r = 0.0
        else:
            bb = sl.bounding_box()
            r = max(abs(bb.min.X), abs(bb.max.X), abs(bb.min.Y), abs(bb.max.Y))
        zs.append(z); rs.append(r)
    return np.array(zs), np.array(rs)


def slide_on(part, j0, j1, d, tol=0.15):
    """Can a bearing of bore d reach the journal z ∈ [j0, j1] from either end of the part? Returns (ok, side)."""
    bb = part.bounding_box()
    zb, rb = profile(part, bb.min.Z, j0)
    zt, rt = profile(part, j1, bb.max.Z)
    lim = d / 2 + tol
    below = bool(np.all(rb[:-1] <= lim)) if len(rb) > 1 else True
    above = bool(np.all(rt[1:] <= lim)) if len(rt) > 1 else True
    return below or above, ("−Z end" if below else "+Z end" if above else "blocked both ways")


def cases():
    """(name, part in its axis frame (axis = Z), [(journal z0, z1, bore d)], insertion note or None)."""
    from .parts import arm, base, roll, shoulder
    from .parts import belt_stage as bs
    C = []
    # D19 (2026-11-27): pitch and elbow are one-piece tube + pulley (like yaw, loop 44); roll is still split (roll.py)
    ps = Rot(90, 0, 0) * shoulder.pitch_shaft()           # z = y: the outboard end (double-D, bolted pitch hub) at +Z
    C.append(("pitch shaft (D19: 60T one piece; pitch hub bolted on the outboard double-D, outer wall bolted on after the shaft)", ps,
              [(shoulder.W_IN - 3.0, shoulder.W_IN + 3.0, 40.0), (shoulder.W_OUT - 3.0, shoulder.W_OUT + 3.0, 40.0)], None))
    if roll.INLINE:
        # D24 inline roll: no roll shaft or pulley. The 61806 (in roll_arm_rear) slides onto the roll link's Ø30 stub from −X;
        # only roll-link material inside the bearing's path (r ≤ 25 about the roll axis) can block it.
        from build123d import Cylinder
        st = Rot(0, -90, 0) * Pos(0, -roll.ROLL_Y, 0) * (roll.roll_link() & Pos(0, roll.ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(25, 400))
        C.append(("roll stage, D24 inline GIM8108-36 (no shaft or pulley; 61806 onto the roll link's stub from −X, clevis bolted to the output flange)", st,
                  [(roll.W1[0], roll.W1[1], roll.BRG_R["d"])], None))
    else:
        rs = Rot(0, -90, 0) * Pos(0, -roll.ROLL_Y, 0) * roll.roll_shaft()   # +X → +Z
        C.append(("roll shaft (D19: 80T printed on the tube; bearings from both ends, into the walls' capped U-slots from below; clevis caps on top)", rs,
                  [(roll.W1[0], roll.W1[1], 40.0), (roll.W2[0], roll.W2[1], 40.0)], None))
    ys = Pos(0, -roll.ROLL_Y, 0) * roll.yaw_shaft()
    C.append(("yaw shaft (D19, loop 44: 40T + spring hub one piece; up into the top plate, bottom plate bolted on, flange last)", ys,
              [(roll.P1[0], roll.P1[1], 40.0), (roll.YAW_BRG_B[0], roll.YAW_BRG_B[1], 40.0)], None))
    eh = Rot(90, 0, 0) * arm.elbow_pulley_hub()           # z = y (loop 40 used Rot(-90): z = −y, which mirrored the journals)
    C.append(("elbow shaft (D19: 40T one piece; 2 × 61808 in capped U-seats + 2 × 6708; the cassette goes in from below)", eh,
              [(arm.CAS_A[0], arm.CAS_A[1], 40.0), (arm.CAS_B[0], arm.CAS_B[1], 40.0), (-54 - 3.5, -54 + 3.5, 40.0), (54 - 3.5, 54 + 3.5, 40.0)], None))
    st = dict((n, p) for n, p, _ in bs.small_stage_v3(20))
    C.append(("D13 small stage v3 (D24 bolted cartridge, Oldham hub; steel Ø12 shaft, pulley and bearings separate) at pitch, roll, yaw, elbow", st["shaft_12"],
              [(bs.V3["jA"][0], bs.V3["jA"][1], bs.BRG_6801["d"]), (bs.V3["jB"][0], bs.V3["jB"][1], bs.BRG_6801["d"]),
               (bs.V3["pulley"][0], bs.V3["pulley"][1], bs.SHAFT_D)], None))
    from .parts import hand
    sp = dict((n, p) for n, p, _ in hand.wrist_roll_parts())["wr_spindle"]
    zf = hand.wrist_roll_face_z()
    C.append(("D16 wrist-roll spindle (loop 43: from above, shoulder on the upper bearing; bearings from below, end cap)", sp,
              [(zf - 10.0, zf - 3.0, hand.BRG_WR["d"]), (zf - 17.0, zf - 10.0, hand.BRG_WR["d"])], None))
    ws = base.wheel_shaft()
    C.append(("wheel shaft + flange (×4)", ws, [(-44, -44 + 7, 40.0), (-40 + 8 - 7, -40 + 8, 40.0)], None))
    return C



# ---------------------------------------------------------------------------------------------------------------------
# R14 (c) tool access and (d) cartridge swap (Fable review C3, 2026-11-27). Both run on the whole robot (robot.build) at a
# service pose, left arm, covers off (they are not built), plus the parts listed in REMOVED for the step.
#
# (c) A fastener passes when a real driver reaches it: a Ø6 shank straight along the screw axis for ≥ 80 mm from the head
#     (inside a counterbore the probe is the hex key, Ø2.5, until the counterbore mouth), or a ball-end hex key tilted
#     ≤ 25° (Ø6 shank, 8 azimuths at 15° and 25°). Blocked = the probe overlaps another part by > 0.5 mm³.
# (d) A cartridge swap passes when, with its flange screws out (they pass (c)), the cartridge (sleeve + cap + motor +
#     motor-side Oldham adapter + disc) slides out along its axis, sampled every 2.5 mm, without touching anything that
#     stays (belt, pulley, shaft, bearings, shaft-side Oldham hub, walls): first until it is disengaged (spigot lip and
#     Oldham tongue clear, 6 mm), then either straight on until it has cleared its own length, or sideways (stage ±X/±Y,
#     150 mm) from any disengaged position. The belt and the shaft never move.
SERVICE_POSE = dict(lift=0.75, pitch=0.0, elbow=-90.0, wrist=0.0)   # arm down, forearm level (hand clear of the base)
REMOVED_ALWAYS = ("cover", "boot", "duct")         # D23 covers / boots / ducts come off first for any drive service
REMOVED_AT = {"pitch": ("torso_shell",)}           # the pitch cartridges sit inside the torso: its half-shell comes off
DRIVER = dict(r=3.0, L=80.0, key_r=1.25, tilts=(15.0, 25.0), az=8, tol=0.5)


def _frames(pose=SERVICE_POSE):
    """Location of each module frame in the robot frame (left arm), as robot.build places them."""
    from build123d import Pos, Rot
    from .params import BASELINE as D
    from .parts import arm, roll, shoulder
    a = D.arm
    sh = Pos(a.shoulder_x * 1000, a.shoulder_y * 1000, pose["lift"] * 1000)
    piv = sh * Rot(0, pose["pitch"], 0)
    up = piv * Pos(0, roll.ROLL_Y, -roll.YAW_DROP)
    fe = up * Pos(0, 0, -arm.ELBOW_Z) * Rot(0, pose["elbow"], 0)
    return dict(shoulder=sh * Pos(0, -(shoulder.Y_FER + 18), 0), roll=piv, arm=up, forearm=fe,
                hand=fe * Pos(0, 0, -arm.FORE_L) * Rot(0, pose["wrist"], 0), world=Pos(0, 0, 0))


def _robot(pose=SERVICE_POSE):
    from .parts import robot
    poses = {s: dict(pitch=pose["pitch"], elbow=pose["elbow"], wrist=pose["wrist"]) for s in ("l", "r")}
    parts = [(n, p) for n, p, _ in robot.build(pose["lift"], poses) if not n.startswith("r_")]
    return [(n, p, p.bounding_box()) for n, p in parts]


def _overlap(bb, cb, pad=0.0):
    return (bb.min.X - pad < cb.max.X and bb.max.X + pad > cb.min.X and bb.min.Y - pad < cb.max.Y and bb.max.Y + pad > cb.min.Y
            and bb.min.Z - pad < cb.max.Z and bb.max.Z + pad > cb.min.Z)


def _hits(probe, parts, skip=()):
    bb = probe.bounding_box()
    out = []
    for n, p, pb in parts:
        if any(k in n for k in skip) or not _overlap(bb, pb):
            continue
        try:
            v = (probe & p).volume
        except Exception:
            v = 0.0
        if v > DRIVER["tol"]:
            out.append(n)
    return out


def _driver(T, pt, ax, cb=0.0, tilt=0.0, az=0.0):
    """Driver probe in the robot frame: hex key (r key_r) through cb mm of counterbore, then the Ø6 shank, total L,
    along ax tilted by tilt° toward azimuth az (about the head point)."""
    import math as _m
    import numpy as _np
    from build123d import Align, Cylinder, Plane
    u = _np.array(ax, float) / _np.linalg.norm(ax)
    if tilt:
        x = _np.cross(u, [0, 0, 1] if abs(u[2]) < 0.9 else [1, 0, 0]); x /= _np.linalg.norm(x)
        y = _np.cross(u, x)
        t, f = _m.radians(tilt), _m.radians(az)
        u = _m.cos(t) * u + _m.sin(t) * (_m.cos(f) * x + _m.sin(f) * y)
    MINZ = (Align.CENTER, Align.CENTER, Align.MIN)
    p0 = _np.array(pt, float) + 0.3 * u
    pr = None
    if cb > 0.3:
        pr = Plane(origin=tuple(p0), z_dir=tuple(u)).location * Cylinder(DRIVER["key_r"], cb, align=MINZ)
    p1 = p0 + max(cb, 0.0) * u
    sh = Plane(origin=tuple(p1), z_dir=tuple(u)).location * Cylinder(DRIVER["r"], DRIVER["L"] - max(cb, 0.0), align=MINZ)
    return T * (sh if pr is None else pr + sh)


def ear_sites(fr=None):
    """(joint, stage→robot Location, motor, ear site) of every bolted cartridge. The site is the module's EAR_SITE (the
    pattern its CAD passes to belt_stage), "default" when the module names none."""
    from .parts import arm, cartridge_m as cm, roll, shoulder, upper
    fr = fr or _frames()
    s = lambda mod: getattr(mod, "EAR_SITE", "default")
    out = [("pitch", fr["shoulder"] * shoulder._ST, shoulder.MOT, s(shoulder)), ("elbow", fr["arm"] * arm.STAGE, arm.MOT, s(arm)),
           ("yaw", fr["roll"] * roll.YAW_T, cm.MOTOR, s(roll)), ("lift", upper.lift_stage_T(), cm.MOTOR, s(upper))]
    if not roll.INLINE:
        out.append(("roll (belt)", fr["roll"] * roll.ROLL_T, cm.MOTOR, s(roll)))
    return out


def fasteners(fr=None):
    """(joint, label, frame Location, point, axis, counterbore depth, removed-for-this-step name keys, kind) for every drive
    train fastener R14 (c) covers. kind 'swap' = undone for a cartridge swap; 'service' = other drive-train service."""
    import math as _m
    from build123d import Pos, Rot
    from .parts import arm, cartridge_m as cm, hand, roll, shoulder, upper
    fr = fr or _frames()
    out = []
    # bolted cartridge ears (D24 v2): pitch, elbow, yaw, lift (+ the belt roll when PIP_ROLL_DRIVE=belt)
    for j, T, m, site in ear_sites(fr):
        for lab, pt, ax in cm.ear_fasteners(m, site):
            out.append((j, f"flange {lab} (M4, site {site})", T, pt, ax, 0.0, (), "swap"))
    # pitch (C1): the 4 HUB_TIE nuts on the clamp ring in the detach-face recess (arm detached; the studs are Loctited into
    # the crank, nothing to undo inboard), outer wall (3 × M4 into the web)
    ht = shoulder.HUB_TIE
    for a in shoulder._tie_angles():
        x, z = ht["r"] * _m.cos(a), ht["r"] * _m.sin(a)
        out.append(("pitch", f"pitch hub tie nut M4 {_m.degrees(a):.0f}° (arm detached)", fr["shoulder"], (x, shoulder.Y_FER + 18 - ht["face_gap"], z), (0, 1, 0),
                    0.0, ("l_roll_", "l_arm_"), "service"))
    for z in shoulder.WEB_INSERTS_Z:
        out.append(("pitch", f"outer wall M4 z {z:.0f}", fr["shoulder"], (shoulder.WEB_X, shoulder.W_OUT + shoulder.WALL_T / 2 + 4.0, z), (0, 1, 0), 0.0, (), "service"))
    # elbow: bolted seat caps from below (bone cap off), forearm pinch clamp (2 × M5, head on either side)
    for side, sc in arm.SEAT_SCREW.items():
        head_h = 4.0 if side == "-" else 3.0
        z_head = arm.CAP_Z + 4.5 + sc["head_z"] - head_h
        for x in (-sc["x"], sc["x"]):
            out.append(("elbow", f"seat cap {side}Y M{4 if side == '-' else 3} x {x:+.0f} (bone cap off)", fr["arm"], (x, sc["y"], z_head), (0, 0, -1),
                        z_head - (arm.CAP_Z + 0.2), ("bone_cap",), "service"))
    for yy in (arm.FORE_Y - 5.5, arm.FORE_Y + 5.5):
        xs = _m.sqrt(40.0 ** 2 - 29.0 ** 2)
        out.append(("elbow", f"forearm pinch M5 y {yy:.1f}", fr["forearm"], (xs + 5.0, yy, 29.0), (1, 0, 0), 0.0, (), "service"))
        out.append(("elbow", f"forearm pinch M5 y {yy:.1f} (head −X)", fr["forearm"], (-xs - 5.0, yy, 29.0), (-1, 0, 0), 0.0, (), "service"))
    # yaw (C1): flange → tube tie, 6 × M3 from the tube's top end (heads on the clamp ring in the counterbore). Tightened on
    # the bench before the yaw block goes onto the roll output (roll.py YAW_TIE), so probed on the yaw block alone ("only:").
    lv = roll.yaw_tie_levels()
    for a in roll._yaw_tie_angles():
        r = roll.YAW_TIE["r"]
        out.append(("yaw", f"yaw tie M3 {_m.degrees(a):.0f}° (bench: yaw block off the roll output)", fr["roll"],
                    (r * _m.cos(a), roll.ROLL_Y + r * _m.sin(a), lv["head"] + roll.YAW_TIE["head_h"]), (0, 0, 1), 0.0, ("only:l_roll_yaw_",), "service"))
    # wrist: hand off (6 × M3 palm screws from the palm's −Y face), then the can flange (6 × M3 from the hand side)
    for k in range(6):
        a = _m.pi / 3 * k
        out.append(("wrist", f"palm M3 {60 * k}°", fr["hand"], (hand.PALM_SCREW_R * _m.cos(a), -19.5, hand.PALM_SCREW_R * _m.sin(a)), (0, -1, 0), 1.0, (), "swap"))
    for k in range(6):
        a = _m.pi / 3 * k + _m.pi / 6
        r = hand.CAN_FL["bolt_r"]
        out.append(("wrist", f"can flange M3 {_m.degrees(a):.0f}° (hand off)", fr["forearm"], (r * _m.cos(a), arm.WRIST_Y0 - hand.CAN_FL["t"] - 3.0, -arm.FORE_L + r * _m.sin(a)), (0, -1, 0), 0.0,
                    ("l_arm_hand_",), "swap"))
    # inline roll (roll.py, read-only here): 8 × M4 motor → roll link floor, 8 × M5 output adapter → clevis arm
    if roll.INLINE:
        m36 = roll.MOT36
        for k in range(m36["rear_n"]):
            a = 2 * _m.pi * k / m36["rear_n"] + roll.REAR_ANG0
            y, z = roll.ROLL_Y + m36["rear_pcd"] / 2 * _m.cos(a), m36["rear_pcd"] / 2 * _m.sin(a)
            out.append(("roll (inline)", f"motor M4 × 18 {_m.degrees(a):.0f}° (yaw block + rear arm off)", fr["roll"], (roll.X_FLOOR[0], y, z), (-1, 0, 0), 0.0,
                        ("l_roll_yaw_", "l_roll_roll_arm_rear", "l_arm_"), "service"))      # M1: head flush in its counterbore
        for k in range(roll.ADAPT["n"]):
            a = 2 * _m.pi * k / roll.ADAPT["n"] + _m.pi / roll.ADAPT["n"]
            y, z = roll.ROLL_Y + roll.ADAPT["pcd"] / 2 * _m.cos(a), roll.ADAPT["pcd"] / 2 * _m.sin(a)
            out.append(("roll (inline)", f"clevis M5 {_m.degrees(a):.0f}° (spring ring + hub off)", fr["roll"], (roll.X_ARM_F[1], y, z), (1, 0, 0), 0.0,
                        ("l_roll_roll_spring_",), "service"))
    return out


def tool_access(parts=None, fr=None):
    """R14 (c): one row per fastener: (joint, label, kind, ok, how, blockers)."""
    parts = parts if parts is not None else _robot()
    rows = []
    for j, lab, T, pt, ax, cb, removed, kind in fasteners(fr):
        only = [k[5:] for k in removed if k.startswith("only:")]        # bench step: probe on that sub-assembly alone
        removed = tuple(k for k in removed if not k.startswith("only:")) + REMOVED_ALWAYS + REMOVED_AT.get(j, ())
        P = [q for q in parts if not any(k in q[0] for k in removed) and (not only or any(q[0].startswith(k) for k in only))]
        hit = _hits(_driver(T, pt, ax, cb), P)
        if not hit:
            rows.append((j, lab, kind, True, "straight Ø6 × 80", []))
            continue
        how = None
        for tilt in DRIVER["tilts"]:
            for k in range(DRIVER["az"]):
                if not _hits(_driver(T, pt, ax, cb, tilt, 360.0 * k / DRIVER["az"]), P):
                    how = f"ball-end {tilt:.0f}° (az {360 * k // DRIVER['az']}°)"
                    break
            if how:
                break
        rows.append((j, lab, kind, how is not None, how or "blocked", sorted(set(hit))))
    return rows


def _shift(T, d, x=0.0, y=0.0):
    """Location that moves stage-frame parts d along stage +Z (and x, y sideways), expressed in the robot frame."""
    from build123d import Pos
    return T * Pos(x, y, d) * T.inverse()


def cartridge_swap(parts=None, fr=None, step=2.5):
    """R14 (d): (joint, ok, how, blockers) per cartridge: out with the belt on and the shaft in place. The moving set is the
    sleeve, rear cap, motor and the motor-side Oldham adapter; the Oldham disc stays on the shaft-side hub."""
    import math as _m
    from build123d import Pos, Rot
    from .parts import arm, cartridge_m as cm, hand
    parts = parts if parts is not None else _robot()
    fr = fr or _frames()
    jobs = [(j, {"pitch": "l_sh_", "elbow": "l_arm_e_", "yaw": "l_roll_yaw_", "lift": "lift_", "roll (belt)": "l_roll_roll_"}[j], T, m, site)
            for j, T, m, site in ear_sites(fr)]
    wr = fr["forearm"] * Pos(0, 0, -arm.FORE_L) * Pos(0, hand.WRIST_HUB_Y + hand.KX56.gap + hand.cartridge_d_stack(), 0) * Rot(90, 0, 0)
    rows = []
    for j, pre, T, m, site in jobs + [("wrist (hand off)", "l_arm_wrist_", wr, None, None)]:
        gone = REMOVED_ALWAYS + REMOVED_AT.get(j, ())
        if j.startswith("wrist"):                       # can local +Z → −Y: it comes out toward the hand side
            names = [pre + k for k in ("can", "motor", "adapter")]
            stay = [q for q in parts if q[0] not in names and not q[0].startswith("l_arm_hand_")]
            length, dis, exit_a = hand.cartridge_d_stack() + 10.0, 3.0, None
        else:
            names = [pre + k for k in ("can", "can_cap", "motor", "kx_hub_v2")]
            stay = [q for q in parts if q[0] not in names]
            length, dis = cm.stack_height_bolted(m) + 1.5, cm.EXIT_LIFT
            exit_a = cm.EXIT_SITES.get(site)
        stay = [q for q in stay if not any(k in q[0] for k in gone)]
        mov = [p for n, p, _ in parts if n in names]
        if not mov:
            rows.append((j, False, "cartridge parts not found", names))
            continue
        body = mov[0]
        for p in mov[1:]:
            body = body + p
        free, block, d = 0.0, [], 0.5
        while d <= length + 5.0:
            hit = _hits(_shift(T, d) * body, stay)
            if hit:
                block = hit
                break
            free, d = d, d + step
        if free >= length:
            rows.append((j, True, f"straight out {length:.0f} mm along its axis", []))
            continue
        if free < dis:
            rows.append((j, False, f"stuck after {free:.1f} mm (needs {dis:.1f} to clear the tongue and lip)", sorted(set(block))))
            continue
        dirs = ([(-_m.cos(_m.radians(exit_a)), -_m.sin(_m.radians(exit_a)), f"its side exit (slot at {exit_a:.0f}°)")] if exit_a is not None else [])
        dirs += [(1, 0, "stage +X"), (-1, 0, "stage −X"), (0, 1, "stage +Y"), (0, -1, "stage −Y")]
        how, side_block = None, []
        for dx, dy, lab in dirs:
            hit = []
            for s in range(10, 151, 10):
                hit = _hits(_shift(T, dis, dx * s, dy * s) * body, stay)
                if hit:
                    break
            if not hit:
                how = f"{dis:.1f} mm along its axis, then 150 mm sideways through {lab}"
                break
            side_block += hit
        rows.append((j, how is not None, how or f"blocked after {free:.1f} mm along its axis, no sideways exit",
                     [] if how else sorted(set(block + side_block))))
    return rows

def run():
    rows = []
    for name, part, journals, ins in cases():
        res = [slide_on(part, j0, j1, d) for j0, j1, d in journals]
        ok_b = all(r[0] for r in res)
        rows.append(dict(name=name, bearings=[(round(j0, 1), round(j1, 1), d, ok, side) for (j0, j1, d), (ok, side) in zip(journals, res)],
                         ok_bearings=ok_b, insertion=ins, ok=ok_b and ins is None))
    return rows


def main():
    rows = run()
    n_ok = sum(r["ok"] for r in rows)
    L = ["# Assembly-sequence check (generated by `python -m rnd.assembly_seq`)", "", __doc__.split("\n\n", 1)[1].strip(), "",
         f"**{n_ok}/{len(rows)} drive shafts can be assembled as drawn.**", "",
         "| Shaft | Bearings and pulleys that can slide on (Ø = bore) | Insertion | Verdict |", "|---|---|---|---|"]
    for r in rows:
        bl = "; ".join(f"z {j0}…{j1} (Ø{d:.1f}): {'✅' if ok else '❌'} {side}" for j0, j1, d, ok, side in r["bearings"])
        L.append(f"| {r['name']} | {bl} | {r['insertion'] or '✅'} | {'✅' if r['ok'] else '❌'} |")
    parts = _robot()
    tc = tool_access(parts)
    sw = cartridge_swap(parts)
    pose = ", ".join(f"{k} {v:g}" for k, v in SERVICE_POSE.items())
    joints = list(dict.fromkeys(r[0] for r in tc))
    L += ["", "## R14 (c) tool access", "",
          f"Whole robot (rnd/parts/robot.py), left arm, service pose {pose}; covers, boots and ducts off"
          + "".join(f"; at {j}: {', '.join(v)} off" for j, v in REMOVED_AT.items()) + ". A fastener passes with a Ø6 driver straight along "
          f"its axis for {DRIVER['L']:.0f} mm from the head (the Ø2.5 key inside a counterbore), or a Ø6 ball-end tilted "
          f"{' / '.join(f'{t:.0f}' for t in DRIVER['tilts'])}° at {DRIVER['az']} azimuths. 'swap' = undone for a cartridge swap.", "",
          "| Joint | Fasteners | Pass | Failures (blocked by) |", "|---|---|---|---|"]
    for j in joints:
        rs = [r for r in tc if r[0] == j]
        bad = [f"{r[1]} ({', '.join(x.replace('l_', '') for x in r[5][:3])})" for r in rs if not r[3]]
        ball = sum(1 for r in rs if r[3] and r[4].startswith("ball"))
        L.append(f"| {j} | {len(rs)} | {sum(r[3] for r in rs)}/{len(rs)}{f' ({ball} by ball-end)' if ball else ''} {'✅' if not bad else '❌'} | {'; '.join(bad) or '—'} |")
    L += ["", "<details><summary>Every fastener</summary>", "", "| Joint | Fastener | Kind | Result | Blocked by (straight) |", "|---|---|---|---|---|"]
    L += [f"| {j} | {lab} | {k} | {'✅' if ok else '❌'} {how} | {', '.join(b) or '—'} |" for j, lab, k, ok, how, b in tc]
    L += ["", "</details>", "", "## R14 (d) cartridge swap (belt on, shaft in place)", "",
          "Flange screws out (rows above), then the cartridge (sleeve + rear cap + motor + motor-side Oldham adapter; the disc stays "
          "on the shaft-side hub) moves along its axis in 2.5 mm steps: straight out its own length, or 4.5 mm (adapter tongue and spigot "
          "lip clear) and then 150 mm sideways; pitch and lift have a side-exit slot in the flange (cartridge_m.EXIT_SITES).", "",
          "| Joint | Swap | How / blocked by |", "|---|---|---|"]
    L += [f"| {j} | {'✅' if ok else '❌'} | {how}{(' (' + ', '.join(b) + ')') if b else ''} |" for j, ok, how, b in sw]
    (DOCS / "assembly_seq.md").write_text("\n".join(L))
    per = {j: dict(ok=sum(r[3] for r in tc if r[0] == j), n=sum(1 for r in tc if r[0] == j)) for j in joints}
    (DOCS / "assembly_seq.json").write_text(json.dumps({"ok": n_ok, "n": len(rows),
                                                        "tool_access": per, "tool_ok": all(r[3] for r in tc),
                                                        "swap": {j: ok for j, ok, _, _ in sw}, "swap_ok": all(ok for _, ok, _, _ in sw)}))
    print("\n".join(L[4:]))

if __name__ == "__main__":
    main()
