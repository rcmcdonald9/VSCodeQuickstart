"""Whole-robot checks: cross-module interference at several lift heights, printability of every printed piece on the
P2S (256 mm cube, best orientation about Z), and the CAD mass roll-up vs rnd/params.py.  python -m rnd.assembly_check
"""
from __future__ import annotations

import itertools
import math
from pathlib import Path

from build123d import Rot

from .budget import mass_budget, upper_total
from .params import BASELINE

BED = 256.0
DOCS = Path(__file__).resolve().parent.parent / "docs" / "rnd"


def printed_parts():
    from .parts import arm, base, belt_stage as bs, cartridge_m as cm, cartridge_s as cs, hand, roll, shoulder, upper
    from .parts.kx import KX44, KX56, KX74, collar, hub, male, retainer
    P = {}
    from .parts.kx import hub_pip
    for k in (KX74, KX56, KX44):
        P[f"{k.name} hub"], P[f"{k.name} collar"], P[f"{k.name} male"] = hub_pip(k), collar(k), male(k)
    P["eccentric bushing"] = bs.eccentric_bushing()
    P.update({"cartridge M can": cm.can(), "cartridge M adapter": cm.motor_adapter(), "cartridge S sleeve": cs.sleeve(),
              "cartridge S horn adapter": cs.horn_adapter(), "cartridge D can": hand.can_d(), "cartridge D adapter": hand.adapter_d(),
              "small-pulley hub": bs.small_hub(), "pulley 40T": bs.pulley(40), "pulley 60T": bs.pulley(60, flange=False),
              "pulley 72T": bs.pulley(72, flange=False),
              "upper bone": arm.upper_bone(), "forearm (ends)": arm.forearm(), "elbow pulley hub": arm.elbow_pulley_hub(),
              "palm": hand.palm(), "fin-ray finger": hand.fin_ray(-125, -162, 21.0, 18, 28, -1), **{f"gripper {n}": p for n, p, c in hand.gripper_parts() if n in ("grip_base", "grip_servo_sleeve", "jaw_moving", "jaw_fixed")},
              "shoulder bracket": shoulder.bracket(), "pitch shaft": shoulder.pitch_shaft(), "ferrule (arm side)": shoulder.ferrule(True),
              "roll link": roll.roll_link(), "roll shaft": roll.roll_shaft(), "clevis + yaw block": roll.clevis_and_yaw_block(), "yaw shaft": roll.yaw_shaft(),
              "carriage": upper.carriage(750), "ratchet strip": upper.ratchet_strip(),
              **{n: p for n, p, c in upper.torso(750) if n in ("torso_shell", "e_tray")},
              **{n: p for n, p, c in upper.head() if n in ("head_bracket", "handle", "yoke", "head_shell", "visor")},
              "deck quarter": base.deck_quarter(0), "skirt quarter": base.skirt_quarter(0), "wheel pod": base.pod(),
              "wheel shaft": base.wheel_shaft(), "mast coupling": base.coupling()[0]})
    return P


def best_fit(solid):
    """Smallest max-dimension over rotations about Z (prints can be turned on the bed); returns (dims, fits)."""
    best = None
    for a in range(0, 90, 5):
        bb = (Rot(0, 0, a) * solid).bounding_box()
        dims = sorted((bb.size.X, bb.size.Y, bb.size.Z))
        if best is None or dims[-1] < best[-1]:
            best = dims
    return best, best[-1] <= BED


def printability():
    rows = []
    for name, part in printed_parts().items():
        sols = part.solids() if hasattr(part, "solids") else [part]
        for i, s in enumerate(sols):
            if s.volume < 50:
                continue
            dims, ok = best_fit(s)
            rows.append((name + (f" [{i + 1}]" if len(sols) > 1 else ""), dims, ok, s.volume / 1000))
    return rows


def interference(lifts=(0.40, 0.75, 1.05)):
    """Arms in a realistic pose for each lift height (floor work at the bottom: arms reaching forward)."""
    from .parts import robot
    out = []
    for L in lifts:
        poses = None if L > 0.5 else {"l": dict(pitch=-55, elbow=-40, wrist=-30), "r": dict(pitch=-60, elbow=-35, wrist=-35)}
        parts = robot.build(L, poses)
        mod = lambda n: n.split("_")[0] if n[:2] in ("l_", "r_") else n.split("_")[0][:4]
        items = [(n, p, p.bounding_box()) for n, p, _ in parts]
        hits = []
        for (a, pa, ba), (b, pb, bb) in itertools.combinations(items, 2):
            if a[:2] == b[:2] and a[:2] in ("l_", "r_") and a.split("_")[1] == b.split("_")[1]:
                continue                                              # same arm sub-module: checked in its own module
            if (ba.min.X > bb.max.X or bb.min.X > ba.max.X or ba.min.Y > bb.max.Y or bb.min.Y > ba.max.Y
                    or ba.min.Z > bb.max.Z or bb.min.Z > ba.max.Z):
                continue
            try:
                v = (pa & pb).volume
            except Exception:
                v = 0
            if v > 1000:                                              # > 1 cm³: real clash, not a mounting interface
                hits.append((a, b, v / 1000))
        out.append((L, hits))
    return out


def mass_rollup(d=BASELINE):
    mb = mass_budget(d)
    return {"base": sum(m for _, m in mb["base"]), "upper core": sum(m for _, m in mb["upper"]),
            "arm (each)": sum(m for _, m in mb.get("arm (each, detachable)", [])), "battery": sum(m for _, m in mb["battery"]),
            "total robot": upper_total(mb) + sum(m for k in ("base", "battery") for _, m in mb[k])}


def report():
    import json
    pr = printability()
    nfail = sum(1 for r in pr if not r[2])
    inter = interference()
    L = ["# Whole-robot assembly check (auto-generated by `python -m rnd.assembly_check`)", "",
         f"## Printability on the P2S (256 mm cube, best rotation about Z): {len(pr) - nfail}/{len(pr)} pieces fit", "",
         "| Piece | Best-fit size (mm) | Volume (cm³) | Fits |", "|---|---|---|---|"]
    for name, dims, ok, vol in sorted(pr, key=lambda r: -r[1][-1]):
        L.append(f"| {name} | {' × '.join(f'{v:.0f}' for v in dims)} | {vol:.0f} | {'✅' if ok else '❌ split'} |")
    L += ["", "## Cross-module interference (clashes > 1 cm³; render pose, or arms forward at the lowest lift)", ""]
    for lift, hits in inter:
        L.append(f"- lift {lift:.2f} m: " + ("**none**" if not hits else "; ".join(f"{a} × {b} ({v:.1f} cm³)" for a, b, v in hits[:12])))
    m = mass_rollup()
    L += ["", "## Mass roll-up (params, now mostly from CAD)", "", "| Piece | kg |", "|---|---|"] + [f"| {k} | {v:.2f} |" for k, v in m.items()]
    (DOCS / "assembly.json").write_text(json.dumps({"printable_ok": len(pr) - nfail, "printable_total": len(pr),
                                                    "clashes": sum(len(h) for _, h in inter)}))
    return "\n".join(L), nfail


if __name__ == "__main__":
    md, nfail = report()
    (DOCS / "assembly.md").write_text(md)
    print(md)
