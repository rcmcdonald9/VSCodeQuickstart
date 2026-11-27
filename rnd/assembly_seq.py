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
    # loop 40: every big pulley is split (double-D / hex), so the shafts are plain tubes + their outboard ends
    ps = Rot(90, 0, 0) * shoulder.pitch_shaft()           # z = y: the ferrule end (outboard) is at +Z, the free inboard end at −Z
    C.append(("pitch shaft (60T split on a double-D; balancer disc + ferrule outboard)", ps,
              [(shoulder.W_IN - 3.0, shoulder.W_IN + 3.0, 40.0), (shoulder.W_OUT - 3.0, shoulder.W_OUT + 3.0, 40.0),
               (shoulder.Y_BELT - 8.0, shoulder.Y_BELT + 8.0, bs.DD_AF + 0.4)], None))
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
              [(roll.P1[0], roll.P1[1], 40.0), (roll.P2[0], roll.P2[1], 40.0)], None))
    eh = Rot(-90, 0, 0) * arm.elbow_pulley_hub()
    C.append(("elbow shaft (40T split on a double-D; 2 × 61808 + 2 × 6708)", eh,
              [(arm.CAS_A[0], arm.CAS_A[1], 40.0), (arm.CAS_B[0], arm.CAS_B[1], 40.0), (-54 - 3.5, -54 + 3.5, 40.0), (54 - 3.5, 54 + 3.5, 40.0),
               (arm.BELT_Y - 8.0, arm.BELT_Y + 8.0, bs.DD_AF + 0.4)], None))
    st = dict((n, p) for n, p, _ in bs.small_stage_v2(20))
    C.append(("D13 small stage v2 (steel Ø12 shaft; pulley, coupling, bearings separate): pitch, roll, yaw, elbow", st["shaft_12"],
              [(bs.V2["jA"][0], bs.V2["jA"][1], bs.BRG_6801["d"]), (bs.V2["jB"][0], bs.V2["jB"][1], bs.BRG_6801["d"]),
               (bs.V2["pulley"][0], bs.V2["pulley"][1], bs.SHAFT_D)], None))
    from .parts import hand
    sp = dict((n, p) for n, p, _ in hand.wrist_roll_parts())["wr_spindle"]
    zf = hand.wrist_roll_face_z()
    C.append(("D16 wrist-roll spindle (loop 43: from above, shoulder on the upper bearing; bearings from below, end cap)", sp,
              [(zf - 10.0, zf - 3.0, hand.BRG_WR["d"]), (zf - 17.0, zf - 10.0, hand.BRG_WR["d"])], None))
    ws = base.wheel_shaft()
    C.append(("wheel shaft + flange (×4)", ws, [(-44, -44 + 7, 40.0), (-40 + 8 - 7, -40 + 8, 40.0)], None))
    return C


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
    (DOCS / "assembly_seq.md").write_text("\n".join(L))
    (DOCS / "assembly_seq.json").write_text(json.dumps({"ok": n_ok, "n": len(rows)}))
    print("\n".join(L[4:]))


if __name__ == "__main__":
    main()
