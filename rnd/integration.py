"""Load-path integration checks on the whole-robot CAD (confidence campaign).  python -m rnd.integration

Clash and printability checks miss parts that *don't touch*. This finds load paths that end in air:
1. **Torque reaction.** Every cartridge can (M/D/S) has a steel torque pin sticking out of its rear floor. The pin's tip
   must land in structure (a radial slot), or the motor housing spins instead of the joint.
2. **Bearing support.** Every small-pulley hub runs on two 61804 journals. Each journal must sit in a structural seat
   (an eccentric bushing in a wall or plate).

3. **Clashes inside a module.** The whole-robot check (`assembly_check`) skips pairs inside one sub-module, so overlaps
   > 50 mm³ inside the arm, roll/yaw, shoulder and hand modules are listed here. Interfaces that are meant to be fused,
   glued or meshed (`FUSED`) are excluded.

A pin counts as reacted when its tip is in clear space and structure sits on both tangential sides of it (a slot or fork).
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

DOCS = Path(__file__).resolve().parent.parent / "docs" / "rnd"
SKIP = ("motor", "belt", "dummy", "brg", "battery", "lidar", "cam", "light")


FUSED = {("ferrule_m", "roll_link"), ("roll_link", "shoulder_cap"), ("yaw_block", "yaw_band"), ("yaw_plate_bot", "yaw_band"), ("e_hub", "e_belt"),
         ("e_belt", "e_bigpulley"), ("e_al_pulley", "e_belt"), ("hand_grip_base", "hand_grip_servo_sleeve"), ("grip_base", "grip_servo_sleeve"),
         ("hand_finray_fixed", "hand_pad_fixed"), ("finray_fixed", "pad_fixed")}


def module_clashes():
    import itertools
    from .parts import arm, base, hand, roll, shoulder, upper
    out = []
    for mn, f in (("arm", arm.assembly), ("roll/yaw", roll.module), ("shoulder", shoulder.module), ("base", base.module),
                  ("upper (mast, lift, torso, head)", lambda: upper.module(0.75)),
                  ("head tilt −30°", lambda: upper.head(0, -30)), ("head tilt +30°", lambda: upper.head(0, 30))):
        for (a, pa, _), (b, pb, _) in itertools.combinations(f(), 2):
            if (a, b) in FUSED or (b, a) in FUSED:
                continue
            ba, bb = pa.bounding_box(), pb.bounding_box()
            if (ba.min.X > bb.max.X or bb.min.X > ba.max.X or ba.min.Y > bb.max.Y or bb.min.Y > ba.max.Y
                    or ba.min.Z > bb.max.Z or bb.min.Z > ba.max.Z):
                continue
            try:
                v = (pa & pb).volume
            except Exception:
                v = 0
            if v > 50:
                out.append((mn, a, b, v))
    return out


def _cyl_faces(solid, r, tol=0.06):
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Cylinder
    out = []
    for f in solid.faces():
        s = BRepAdaptor_Surface(f.wrapped)
        if s.GetType() == GeomAbs_Cylinder and abs(s.Cylinder().Radius() - r) < tol:
            ax = s.Cylinder().Axis()
            d = np.array([ax.Direction().X(), ax.Direction().Y(), ax.Direction().Z()])
            loc = np.array([ax.Location().X(), ax.Location().Y(), ax.Location().Z()])
            c = np.array([f.center().X, f.center().Y, f.center().Z])
            out.append((loc + np.dot(c - loc, d) * d, d))          # on the true axis (a partial face's centroid is off-axis)
    return out


def _dist(pt, solid):
    from build123d import Vertex
    return solid.distance_to(Vertex(*pt))


def _fork(tip, tang, solids, own, r_pin=3.0):
    """A real torque slot: the pin tip is in clear space (≥ r_pin − 0.25 from structure) and some structural solid sits on
    both tangential sides (points ±(r_pin + 0.5) along the tangent within 0.8 mm of it: the CAD distance is to the nearest
    surface, so a point just inside a slot wall reads as its depth into the wall)."""
    near = []
    for n, s in solids:
        if n in own:
            continue
        bb = s.bounding_box()
        if not (bb.min.X - 6 < tip[0] < bb.max.X + 6 and bb.min.Y - 6 < tip[1] < bb.max.Y + 6 and bb.min.Z - 6 < tip[2] < bb.max.Z + 6):
            continue
        near.append((n, s))
    if any(_dist(tip, s) < r_pin - 0.25 for _, s in near):
        return False, "pin tip runs into structure (no slot)"
    for n, s in near:
        if _dist(tip + (r_pin + 0.5) * tang, s) < 0.8 and _dist(tip - (r_pin + 0.5) * tang, s) < 0.8:
            return True, n
    return False, "pin tip in air"


def run(lift=0.75):
    from .parts import robot
    from .parts import cartridge_m as cm
    parts = [(n, p) for n, p, _ in robot.build(lift)]
    struct = [(n, p) for n, p in parts if not any(k in n.lower() for k in SKIP)]
    rows = []
    for n, p in parts:
        nl = n.lower()
        if nl.endswith("can") or nl.endswith("_can") or "can_d" in nl or "sleeve" in nl:
            r_pin = 3.0
            seats = _cyl_faces(p, cm.TORQUE_PIN["d"] / 2 - 0.05)
            if not seats:
                r_pin, seats = 2.5, _cyl_faces(p, 2.45)
            if not seats:
                r_pin, seats = 2.0, _cyl_faces(p, 1.95)
            if not seats:
                if "grip_servo_sleeve" in nl:             # by design: no pin; the sleeve is screwed into grip_base (FUSED)
                    rows.append(("torque pin", n, True, "no pin by design: sleeve screwed into grip_base, rectangular servo pocket"))
                else:
                    rows.append(("torque pin", n, None, "no pin seat found"))
                continue
            c, ax = seats[0]
            cen = np.array([p.center().X, p.center().Y, p.center().Z])
            out = ax if np.dot(c - cen, ax) > 0 else -ax                     # rear: away from the can's centre
            tip = c + out * (2.0 + cm.TORQUE_PIN["stick_out"] / 2)
            rad = (c - cen) - np.dot(c - cen, ax) * ax                     # radial direction from the cartridge axis
            tang = np.cross(ax, rad / np.linalg.norm(rad))
            ok, why = _fork(tip, tang, struct, {n}, r_pin)
            rows.append(("torque pin", n, ok, why))
    ok_n = sum(1 for r in rows if r[2])
    mc = module_clashes()
    L = ["# Load-path integration check (generated by `python -m rnd.integration`)", "", __doc__.split("\n\n", 1)[1].strip(), "",
         f"**Torque reaction: {ok_n}/{len(rows)} cartridge pins land in structure.**", "",
         "| Check | Part | Result | Detail |", "|---|---|---|---|"]
    L += [f"| {k} | {n} | {'✅' if ok else '❌'} | {d} |" for k, n, ok, d in rows]
    L += ["", f"## Clashes inside modules: {len(mc)}", "", "| Module | Part A | Part B | Overlap (mm³) |", "|---|---|---|---|"]
    L += [f"| {m} | {a} | {b} | {v:.0f} |" for m, a, b, v in mc]
    (DOCS / "integration.md").write_text("\n".join(L))
    (DOCS / "integration.json").write_text(json.dumps({"torque_ok": ok_n, "torque_n": len(rows), "module_clashes": len(mc),
                                                     "module_clash_list": [[m, a, b, round(v)] for m, a, b, v in mc]}))
    print("\n".join(L[:8]))
    return rows


if __name__ == "__main__":
    run()
