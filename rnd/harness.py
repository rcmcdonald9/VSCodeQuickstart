"""Cable routing and harness kit (queue item 1, loop 32).  python -m rnd.harness

The question: can the arm bundle cross every joint over its full range without a slip ring? Two methods are checked per
joint (all cable data ESTIMATE until the datasheets are in hand):

1. **Torsion through the hollow shaft.** The bundle runs along the axis and twists. A torsion-rated robot cable allows
   about ±360°/m (TORSION_DEG_PER_M), so the free axial length L must be ≥ (range / 2) / 360 m.
2. **Clock-spring spiral** (as in a car's steering-wheel clock spring). The bundle lies in a flat spiral in an annular
   cavity r_in … r_out around the axis. Rotation winds or unwinds it, so the cable bends but never twists. The capacity
   for a spiral of length S is Δθ = S·(1/r_in − 1/r_out) rad (wound tight on r_in ↔ loose on r_out). It needs
   r_in ≥ the bend radius of the stiffest cable (10 × OD for continuous flex) and a cavity of the bundle's height.

The joint ranges come from the twin (rnd/twin.py JR). The free lengths and the cavity sizes are read off the CAD
(module dimensions; ESTIMATE ± 10 mm).
"""
from __future__ import annotations

import json
import math
from pathlib import Path

DOCS = Path(__file__).resolve().parent.parent / "docs" / "rnd"
TORSION_DEG_PER_M = 360.0          # torsion-rated robot cable class (ESTIMATE: check the datasheet)
BEND_K = 10.0                      # continuous-flex bend radius = 10 × OD (ESTIMATE)

# cable catalogue: (count, OD mm). D14 (loop 40): past the shoulder the arm bundle is 24 V + CAN + USB only; the elbow
# sensor node makes 12 V (buck) and drives the TTL servo bus locally, so 12 V/TTL no longer cross pitch, roll and yaw.
CABLES = {"24 V power pair (2 × 20 AWG silicone, OD 1.8)": (2, 1.8), "CAN twisted pair (26 AWG)": (2, 1.0),
          "USB 2.0 slim 4-core high-flex (OD 2.2; D12)": (1, 2.2)}
ELBOW_ON = {"12 V + GND + TTL (servos, 24 AWG)": (3, 1.2)}      # from the elbow node onward
STOCK_USB_OD = 3.8                  # the camera's stock USB cable (what loop 32 first checked)


def _roll_yaw():
    from .parts import roll
    return roll.ROLL_SPR, roll.YAW_SPR, roll.CAV_H


_RS, _YS, CAV_H = _roll_yaw()
# per joint crossed by the arm bundle: range (deg, from twin JR), free axial length (m) for torsion, spiral cavity
# (r_in, r_out, mm), spiral length S (m), and where. Roll and yaw cavities are real CAD (rnd/parts/roll.py, loop 40).
JOINTS = [
    # pitch: bundle from the torso e-tray through the pitch shaft bore (Ø30) to the detach plug on the ferrule
    ("shoulder pitch", 260, 0.20, (22, 60), 0.60, "inside the torso, around the pitch shaft between the bracket and the torso wall"),
    ("shoulder roll", 115, 0.10, _RS, 0.25, "CAD: clock-spring cassette on the +X end of the roll shaft, outboard of the clevis arm (roll_spring_hub / static ring on a bar to the roll-link boss)"),
    ("upper-arm yaw", 300, 0.0, _YS, 0.40, "CAD: clock-spring cassette between the 40T yaw pulley's hub and the bottom yaw plate (yaw ±150°, hard stops ±155°)"),
    ("elbow", 150, 0.13, (22, 50), 0.60, "elbow housing, +Y side, between cassette plate B (y 26) and the +Y wall (y 53)"),
    ("wrist pitch", 220, 0.06, (16, 30), 0.40, "wrist can end, around the DM-J4310 (Ø57 can: little room)"),
    ("wrist roll", 360, 0.03, (14, 24), 0.35, "palm, around the STS3215 wrist-roll servo"),
]


def bundle(name):
    """Cables crossing a joint. Past the wrist only 12 V/TTL (gripper servo) + I²C (tactile, ToF) + the camera FFC go on."""
    if name.startswith("wrist"):        # D12: the camera is MIPI-CSI on a 0.3 mm FFC through the wrist (bend r ~2 mm); bridge on the forearm
        return {"12 V + GND + TTL (servos, 24 AWG)": (3, 1.2), "I²C 4-core (tactile + ToF)": (4, 0.8), "CSI FFC 15-way (camera)": (1, 0.3)}
    b = dict(CABLES)
    if name == "elbow":
        b.update(ELBOW_ON)
    return b


def analyse():
    rows = []
    for name, rng, L, (ri, ro), S, where in JOINTS:
        b = bundle(name)
        od_max = max(od for _, od in b.values())
        r_bend = BEND_K * od_max
        torsion_ok = L > 0 and (rng / 2) <= TORSION_DEG_PER_M * L
        ri_eff = max(ri, r_bend)
        cap = math.degrees(S * (1 / (ri_eff / 1000) - 1 / (ro / 1000))) if ro > ri_eff else 0.0
        width = sum(n * od for n, od in b.values()) if not name.startswith("wrist") else None
        build = S * 1000 * od_max / (2 * math.pi * ri_eff)                 # radial build when wound tight on r_in (mm)
        fits = build <= (ro - ri_eff) * 0.6 and (width is None or name not in ("shoulder roll", "upper-arm yaw") or width <= CAV_H - 1.0)
        spiral_ok = cap >= rng * 1.2 and fits
        od_nousb = max(max(od for _, od in b.values()), STOCK_USB_OD)
        ri2 = max(ri, BEND_K * od_nousb)
        cap2 = math.degrees(S * (1 / (ri2 / 1000) - 1 / (ro / 1000))) if ro > ri2 else 0.0
        rows.append(dict(joint=name, range=rng, L=L, torsion_ok=torsion_ok, need_L=rng / 2 / TORSION_DEG_PER_M,
                         r_bend=r_bend, cavity=(ri, ro), S=S, spiral_cap=cap, spiral_ok=spiral_ok, build=build, width=width,
                         cap_nousb=cap2, ok_nousb=cap2 >= rng * 1.2, where=where,
                         area=sum(n * math.pi * od ** 2 / 4 for n, od in b.values())))
    return rows


def main():
    rows = analyse()
    n_ok = sum(r["torsion_ok"] or r["spiral_ok"] for r in rows)
    n_ok2 = sum(r["torsion_ok"] or r["ok_nousb"] for r in rows)        # with the stock USB cable
    L = ["# Cable routing across the joints (generated by `python -m rnd.harness`)", "", __doc__.split("\n\n", 1)[1].strip(), "",
         f"**Joints the arm bundle can cross: {n_ok}/{len(rows)} with D12** (CSI flat-flex through the wrist + slim Ø2.2 USB from the forearm); "
         f"{n_ok2}/{len(rows)} with the camera's stock Ø3.8 USB cable.", "",
         "| Joint | Range | Torsion: free length / needed | Spiral cavity r_in–r_out (mm) | Bend r (10×OD) | Spiral S / capacity | Ribbon build / width | Method | Stock USB |",
         "|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        m = "torsion ✅" if r["torsion_ok"] else ("clock spring ✅" if r["spiral_ok"] else "❌")
        L.append(f"| {r['joint']} | {r['range']}° | {r['L'] * 1000:.0f} / {r['need_L'] * 1000:.0f} mm {'✅' if r['torsion_ok'] else '❌'} | "
                 f"{r['cavity'][0]}–{r['cavity'][1]} | {r['r_bend']:.0f} mm | {r['S']:.2f} m / {r['spiral_cap']:.0f}° (need {r['range'] * 1.2:.0f}°) | "
                 f"{r['build']:.1f} mm / {'—' if r['width'] is None else f"{r['width']:.1f} mm"} | {m} | "
                 f"{'✅' if r['torsion_ok'] or r['ok_nousb'] else '❌'} {r['cap_nousb']:.0f}° |")
    L += ["", "**Findings (loop 32):**",
          "- **No joint can rely on torsion through its bore.** The free axial lengths are 30–200 mm, and the ranges need 160–500 mm at ±360°/m. "
          "The old electrical note ('cables through the hollow shafts, no slip rings') needs **clock-spring spirals** at each joint.",
          "- **The camera's stock Ø3.8 USB cable** needs a 38 mm bend radius and fits through none of the wrist joints. **D12:**",
          "  - a MIPI-CSI camera on a thin FFC through both wrist joints",
          "  - a CSI→USB (UVC) bridge board on the forearm",
          "  - a slim 4-core high-flex USB cable (OD ≤ 2.2 mm) from there on",
          "- **Loop 40: roll and yaw cassettes are now real CAD** (external review finding 4). Yaw is back to **±150°** (loop 33 had cut it to ±70°, "
          "which cost side grasps 68 → 40 %):",
          "  - **yaw:** a 9.5 mm cavity between the split 40T pulley's hub (rotating inner wall, r 22.5) and the bottom yaw plate, static ring r 40 screwed to the plate; the ribbon's inner end enters the yaw-shaft bore and runs down the neck",
          "  - **roll:** a cassette on the +X end of the roll shaft, outboard of the clevis arm; the static ring hangs on a bar that runs inboard at z ≈ 0 (a direction the roll output never sweeps) to the roll-link boss",
          "  - **D14:** the elbow sensor node makes 12 V and drives the TTL servo bus, so only 24 V + CAN + USB (a 7.8 mm flat ribbon) cross pitch, roll and yaw",
          "- The ribbon: the round cables side by side on a PET/Kapton carrier tape, wound as in a car's clock spring (cable bends, never twists).",          "",
          "## Harness kit (pre-made, one per arm + torso + base)",
          "| Segment | Contents | Length (ESTIMATE) | Ends |", "|---|---|---|---|",
          "| Torso e-tray → shoulder detach plug | 24 V pair, CAN pair, 12 V/TTL, USB | 0.45 m + 0.6 m pitch spiral | XT30 + 6-pin pogo / USB-C |",
          "| Detach plug → elbow sensor node | same | 0.40 m + 3 × 0.6 m spirals (roll, yaw, elbow) | daisy-chain JST-GH / VH at each cartridge |",
          "| Elbow node → hand | 12 V/TTL, I²C, camera (FFC per option 1) | 0.35 m + 2 × 0.3 m spirals (wrist) | JST-GH / FFC |",
          "| Mast: torso → head | 24 V (F 1 A), CAN A, TTL 3 | 0.75 m in a cable chain along the lift travel | JST-GH |",
          "| Base ↔ upper coupling | XT60 + 6-pin pogo (CAN A, e-stop loop) | 0.3 m | blind-mate |"]
    (DOCS / "harness.md").write_text("\n".join(L))
    (DOCS / "harness.json").write_text(json.dumps({"ok": n_ok, "n": len(rows), "ok_nousb": n_ok2}))
    print("\n".join(L[4:15]))


if __name__ == "__main__":
    main()
