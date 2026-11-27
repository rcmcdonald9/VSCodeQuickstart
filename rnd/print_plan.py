"""Print plan: every printed part × quantity, sliced with PrusaSlicer (generic profile, PETG, 0.2 mm, 4 walls, 25 % infill).
python -m rnd.print_plan   (slices ~70 parts, 2 at a time; unchanged parts come from rnd/out/print/slice_cache.json)

The P2S runs far faster than PrusaSlicer's generic printer profile. P2S time ≈ generic × P2S_FACTOR (ESTIMATE; refine after
the first real print).
"""
from __future__ import annotations

import hashlib
import json
import re
import subprocess
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from build123d import export_stl

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "out" / "print"
DOCS = ROOT.parent / "docs" / "rnd"
P2S_FACTOR = 0.35       # ESTIMATE: Bambu-class speed/accel vs the generic profile
WORKERS = 2

# (part key in assembly_check.printed_parts() or EXTRA below, quantity, note). Wave-1 design (2026-11-27): D24 bolted arm
# cartridges (no KX socket on the arm; D15 retired there), D19 one-piece shaft-pulleys with capped seats, D16 wrist spindle.
# W10 (2026-11-27): the inline roll is the default (owner OK; the belt-roll rows apply with PIP_ROLL_DRIVE=belt, BELT_ROLL
# below), the lift is a bolted D13/D24 stage (lift v2: no KX74 socket or hub), and the printed ASA Oldham motor tongue is
# gone (W8b: rejected at 0.74× friction; the motor-side hub is 6061, a BOM line). D26 (wave 11): no Oldham, so no eccentric
# bushings: one bonded wall-B bushing per direct stage, a printed roller in each pitch idler, a printed key in each pitch exit slot.
QTY = {
    "cartridge M can (bolted, D24)": (7, "pitch, yaw, elbow × 2 arms + the lift: Ø86.6 sleeve + flange with 4 ear lobes per site (cartridge_m.EAR_SITES; one is sliced) (ASA, A6)"),
    "cartridge M rear cap (D24)": (7, "carries the motor (3 × M3), 3 radial M3 into the sleeve"),
    "cartridge M can": (4, "KX cartridges: the 4 wheels"), "cartridge M adapter": (4, "KX74 male face (wheels)"),
    "KX74 hub": (4, "wheel hubs, bolted to the GIM output (D17)"),
    "wall-B bearing bushing (D26)": (7, "1 per direct stage: 6 arm + lift, bonded in wall B round the 6001 (lift: 61901, 6 mm wide)"),
    "pitch idler roller (D26)": (2, "flat roller Ø25 × 16 with 1.5 mm flanges on 2 × 625-2RS (PETG-CF)"),
    "pitch exit-slot key (D26)": (2, "fills the inner wall's exit slot under the 6061 strap; mirror for the right arm"),
    "cartridge D can": (2, "wrist pitch DM-J4310, bolted flange (D24: the KX56 went)"), "cartridge D adapter": (2, ""),
    "KX44 hub": (2, "head pan + tilt (D16: the wrist roll lost its KX44)"), "KX44 collar": (2, ""),
    "cartridge S sleeve": (4, "wrist roll × 2, head pan + tilt"), "cartridge S horn adapter": (4, ""),
    "upper bone": (2, ""), "upper bone cap": (2, "screw-on bottom cap: the cassette goes in from below"),
    "elbow seat cap A (+Y)": (2, "D19 bolted seat cap, 2 × M3 into lug inserts"), "elbow seat cap B (−Y)": (2, "D19 bolted seat cap, 2 × M4"),
    "elbow cassette plate": (2, "plate B (anti-rotation pin tab)"), "elbow cassette plate A": (2, "the elbow cartridge bolts to it (D24)"),
    "elbow cassette spacer": (4, ""), "forearm (ends)": (2, "elbow end + clamp, wrist rim with M3 inserts, CF sockets"), "elbow pulley hub": (2, ""),
    "palm": (2, ""), "gripper grip_base": (2, ""), "gripper grip_servo_sleeve": (2, ""), "gripper jaw_moving": (2, ""), "gripper jaw_fixed": (2, ""),
    "shoulder bracket body": (2, "mirror for the right arm"), "shoulder bracket outer wall": (2, "bolted print (D19: the pitch shaft drops in)"),
    "pitch shaft (integral 60T)": (2, "D19 one piece: Ø40 tube + 60T"), "pitch hub (balancer disc + ferrule)": (2, "bolted to the shaft end"),
    "ferrule (arm side)": (2, ""), "roll link": (2, "inline roll link (GIM8108-36 seat, D24)"), "roll arm rear": (2, "inline roll rear support arm (61806 seat, W8c stop bushes)"),
    "clevis + yaw block": (2, ""), "yaw shaft": (2, "one-piece tube + 40T"), "yaw flange": (2, "bolted to the yaw tube (4 × M4)"),
    "yaw plate bottom": (2, "separate bolted print (loop 44)"), "roll spring ring": (2, ""),
    "carriage": (1, ""), "ratchet strip": (1, "4 segments"), "torso_shell": (1, "2 halves"), "e_tray": (1, ""),
    "head_bracket": (1, ""), "handle": (1, ""), "yoke": (1, "D22 fork to the tilt pivot"), "head_shell": (1, ""), "visor": (1, "dark PETG"),
    "head_pivot_stub": (1, "D22 passive pivot (6805)"), "head_drive_stub": (1, "D22 tilt drive disc"), "lift cover": (1, "R17 v3, closed"),
    **{f"cover {n}": (2, "R17/D23, mirror for the right arm") for n in ("elbow_bay", "collar_door", "forearm_f", "forearm_r", "elbow_cap", "wrist_f", "wrist_r")},
    **{f"cover {n}": (1, "R17/D23") for n in ("mast_cap", "collar_bot", "collar_top", "spine_pod", "back_plate", *(f"liner_w{a}" for a in (0, 90, 180, 270)), *(f"belly_{q}" for q in range(4)))},
    **{f"cover belt_duct [{i}/5]": (1, "R17 belt duct segment") for i in range(1, 6)},
    "fin-ray finger": (4, "TPU 95A, 2 per gripper"),
    "deck quarter": (4, ""), "skirt quarter": (4, ""), "wheel pod": (4, ""), "wheel shaft": (4, ""), "dock_wing_l": (2, "self-docking charger: left + mirrored right wing"), "dock_tower": (1, "dock contact tower"), "torque hanger (wheel)": (4, "standard torque-reaction part, screws under the deck"), "mast coupling": (1, ""),
}
# Belt-roll fallback (PIP_ROLL_DRIVE=belt): its roll rows, swapped in for the inline ones at import.
BELT_ROLL = {"cartridge M can (bolted, D24)": (9, "pitch, roll, yaw, elbow × 2 arms + the lift"), "cartridge M rear cap (D24)": (9, ""),
             "wall-B bearing bushing (D26)": (9, "1 per direct stage: 8 arm + lift"), "roll link": (2, ""), "roll shaft + 80T (one piece)": (2, "D19"),
             "roll seat cap": (4, "2 per arm"), "clevis cap": (4, "inner + outer clevis caps, pinch bolt (the inner one stands in for both)")}
# Printed parts the wave-1 design retired from the arm (the print plan before D24/D19 had them): what the build drops.
# W10: the 8 printed ASA Oldham tongue adapters (W8b), the lift's KX74 collar + D13 KX74 hub v2 and its KX can/adapter (lift v2).
RETIRED = {"KX74 collar": 9, "KX56 hub": 2, "KX56 collar": 2, "cartridge M can": 9, "cartridge M adapter": 9, "small-pulley hub": 9,
           "shoulder bracket": 2, "pitch shaft": 2, "roll shaft": 2, "cartridge M Oldham adapter (D24)": 8, "D13 KX74 hub v2 (lift)": 1,
           "eccentric bushing": 14}


def _roll_rows():
    from .parts import roll
    if not roll.INLINE:
        QTY.pop("roll arm rear", None)
        QTY.update(BELT_ROLL)


_roll_rows()
# Inline roll (D24 option, PIP_ROLL_DRIVE=inline): per arm, the roll cartridge (bolted can + cap + adapter), the roll
# shaft + 80T, its seat caps and the clevis caps go; the inline roll link and the rear roll arm come in (roll.printed_parts).
INLINE_DELTA = "−2 bolted cans/caps, −2 roll shafts, −4 roll seat caps, −4 clevis caps, −2 wall-B bushings; + 2 rear roll arms (inline roll link)"


# printed parts of the two arms (incl. the torso-side pitch modules): the M1 bench arm prints half of these
ARM_KEYS = {"cartridge M can (bolted, D24)", "cartridge M rear cap (D24)", "wall-B bearing bushing (D26)", "pitch idler roller (D26)",
            "pitch exit-slot key (D26)", "roll arm rear", "clevis cap",
            "cartridge D can", "cartridge D adapter", "cartridge S sleeve", "cartridge S horn adapter", "upper bone", "upper bone cap",
            "elbow seat cap A (+Y)", "elbow seat cap B (−Y)", "elbow cassette plate", "elbow cassette plate A", "elbow cassette spacer",
            "forearm (ends)", "elbow pulley hub", "palm", "gripper grip_base", "gripper grip_servo_sleeve", "gripper jaw_moving",
            "gripper jaw_fixed", "shoulder bracket body", "shoulder bracket outer wall", "pitch shaft (integral 60T)",
            "pitch hub (balancer disc + ferrule)", "ferrule (arm side)", "roll link", "roll shaft + 80T (one piece)", "roll seat cap",
            "clevis cap + yaw torque fork", "clevis + yaw block", "yaw shaft", "yaw flange", "yaw plate bottom", "roll spring ring",
            "fin-ray finger"}


def extra_parts():
    """Printed parts the assembly-check dict doesn't list (seat caps, yaw flange / bottom plate, spring ring, D26 stage parts)."""
    from .parts import arm, belt_stage as bs, roll, shoulder
    v4 = dict((n, p) for n, p, _ in bs.small_stage_v4())
    idl = dict((n, p) for n, p, _ in bs.idler_parts(shoulder.idler()))
    return {"elbow seat cap A (+Y)": arm.elbow_seat_cap("+"), "elbow seat cap B (−Y)": arm.elbow_seat_cap("-"),
            "yaw flange": roll.yaw_flange(), "yaw plate bottom": roll.yaw_plate_bot(), "roll spring ring": roll.roll_spring_ring(),
            "wall-B bearing bushing (D26)": v4["brg_B_bushing"], "pitch idler roller (D26)": idl["idler_roller"],
            "pitch exit-slot key (D26)": shoulder.slot_key()}


def export():
    from .assembly_check import printed_parts
    OUT.mkdir(parents=True, exist_ok=True)
    files = []
    P = printed_parts()
    P.update({k: v for k, v in extra_parts().items() if k not in P})
    missing = [k for k in QTY if k not in P]
    if missing:
        print("print_plan: not in the CAD (skipped):", missing, flush=True)
    for name, part in P.items():
        if name not in QTY:
            continue
        sols = [s for s in (part.solids() if hasattr(part, "solids") else [part]) if s.volume > 50]
        for i, s in enumerate(sols):
            f = OUT / (re.sub(r"[^a-z0-9]+", "_", name.lower()) + (f"_{i + 1}" if len(sols) > 1 else "") + ".stl")
            export_stl(s, str(f))
            files.append((name, f))
    keep = {f for _, f in files}
    for old in OUT.glob("*.stl"):                  # parts the design no longer has
        if old not in keep:
            old.unlink()
    return files


CACHE = OUT / "slice_cache.json"     # sha1(STL) → (generic h, g): only new or changed parts are re-sliced


def slice_one(stl: Path, cache=None):
    key = hashlib.sha1(stl.read_bytes()).hexdigest()
    if cache is not None and key in cache:
        return tuple(cache[key])
    g = stl.with_suffix(".gcode")
    cmd = ["prusa-slicer", "--export-gcode", "--layer-height", "0.2", "--fill-density", "25%", "--perimeters", "4",
           "--support-material", "--support-material-auto", "--filament-type", "PETG", "--filament-density", "1.27",
           "--filament-diameter", "1.75", "--temperature", "240", "--bed-temperature", "80", "--center", "128,128",
           "--bed-shape", "0x0,256x0,256x256,0x256", "--max-print-height", "256", "-o", str(g), str(stl)]
    subprocess.run(cmd, capture_output=True, timeout=900)
    if not g.exists():
        return None, None
    txt = g.read_text(errors="ignore")
    m = re.search(r"estimated printing time \(normal mode\) = (.*)", txt)
    gr = re.search(r"filament used \[g\] = ([\d.]+)", txt)
    h = 0.0
    if m:
        for v, u in re.findall(r"(\d+)([dhms])", m.group(1)):
            h += int(v) * {"d": 24, "h": 1, "m": 1 / 60, "s": 1 / 3600}[u]
    g.unlink()
    res = (h, float(gr.group(1)) if gr else None)
    if cache is not None:
        cache[key] = res
    return res


def main():
    files = export()
    cache = json.loads(CACHE.read_text()) if CACHE.exists() else {}
    with ThreadPoolExecutor(WORKERS) as ex:                # 4 shared cores: 2 slicers
        res = list(ex.map(lambda nf: (nf[0], nf[1].name, *slice_one(nf[1], cache)), files))
    CACHE.write_text(json.dumps(cache, indent=0, sort_keys=True))
    per = {}
    for name, fn, h, g in res:
        q = QTY[name][0]
        a = per.setdefault(name, dict(q=q, h=0.0, g=0.0, files=0, note=QTY[name][1]))
        a["h"] += (h or 0) * q; a["g"] += (g or 0) * q; a["files"] += 1
    tot_h = sum(a["h"] for a in per.values()); tot_g = sum(a["g"] for a in per.values())
    prev = json.loads((DOCS / "print_plan.json").read_text()) if (DOCS / "print_plan.json").exists() else {}
    arm_h = sum(a["h"] for n, a in per.items() if n in ARM_KEYS) / 2; arm_g = sum(a["g"] for n, a in per.items() if n in ARM_KEYS) / 2
    L = ["# Print plan (auto-generated by `python -m rnd.print_plan`)", "",
         "PrusaSlicer generic profile: PETG, 0.2 mm layers, 4 walls, 25 % infill, auto supports. "
         f"P2S time ≈ generic × {P2S_FACTOR} (ESTIMATE until the first print is timed).", "",
         f"**Total: {tot_g / 1000:.1f} kg of PETG (≈ ${tot_g / 1000 * 20:.0f} at $20/kg); {tot_h:.0f} h generic ≈ {tot_h * P2S_FACTOR:.0f} h on the P2S "
         f"(≈ {tot_h * P2S_FACTOR / 24:.1f} days of continuous printing).**", "",
         f"**One arm ≈** (shoulder pitch module + arm + hand, the M1 print): {arm_g / 1000:.2f} kg, {arm_h:.0f} h generic ≈ {arm_h * P2S_FACTOR:.0f} h on the P2S.", ""]
    if prev.get("kg"):
        L += [f"**Since the last plan:** {prev['kg']:.1f} → {tot_g / 1000:.1f} kg, {prev.get('hours_p2s', 0):.0f} → {tot_h * P2S_FACTOR:.0f} h on the P2S"
              + (f"; one arm {prev['arm_hours_p2s']:.0f} → {arm_h * P2S_FACTOR:.0f} h" if prev.get("arm_hours_p2s") else "") + ".", ""]
    L += ["**Wave 1 (D24 bolted cartridges, D19 one-piece shafts):** the arm no longer prints KX74/KX56 hubs + collars or the KX "
          "cartridge adapters (8 KX74 collars, 2 KX56 sockets, 8 KX cans/adapters retired from the arm), nor split pulleys; it prints "
          "bolted cans + rear caps, the one-piece pitch/roll/yaw/elbow shaft-pulleys and the D19 seat caps. "
          f"Inline roll (D24, the default since the owner's OK): {INLINE_DELTA}. **W10 (2026-11-27):** the 8 printed ASA Oldham tongue adapters are gone "
          "(W8b: 0.74× friction at the peak; the motor-side hub is 6061, a BOM line) and the lift is a bolted D13/D24 stage (lift v2: its KX74 collar, "
          "KX74 hub v2 and KX can/adapter go, a bolted can + cap come in). **D26 (wave 11):** the couplings go, so the 14 eccentric bushings do too; "
          "+7 bonded wall-B bushings (one per direct stage), +2 pitch idler rollers, +2 pitch exit-slot keys.", "",
          "| Part | Qty | Filament (g, all) | Generic time (h, all) | Note |", "|---|---|---|---|---|"]
    for name, a in sorted(per.items(), key=lambda kv: -kv[1]["h"]):
        L.append(f"| {name} | {a['q']} | {a['g']:.0f} | {a['h']:.1f} | {a['note']} |")
    md = "\n".join(L)
    (DOCS / "print_plan.md").write_text(md)
    (DOCS / "print_plan.json").write_text(json.dumps({"kg": tot_g / 1000, "hours_generic": tot_h, "hours_p2s": tot_h * P2S_FACTOR,
                                                         "arm_kg": arm_g / 1000, "arm_hours_p2s": arm_h * P2S_FACTOR,
                                                         "covers_kg": sum(a["g"] for n, a in per.items() if n.startswith(("cover ", "lift cover"))) / 1000}))
    print(md)


if __name__ == "__main__":
    main()
