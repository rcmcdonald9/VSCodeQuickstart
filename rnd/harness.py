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

# cable catalogue: (count, OD mm)
CABLES = {"24 V power pair (2 × 18 AWG silicone)": (2, 2.2), "CAN twisted pair (26 AWG)": (2, 1.0),
          "12 V + GND + TTL (servos, 24 AWG)": (3, 1.2), "USB 2.0 slim 4-core high-flex (OD 2.2; D12)": (1, 2.2)}
STOCK_USB_OD = 3.8                  # the camera's stock USB cable (what loop 32 first checked)

# per joint crossed by the arm bundle: range (deg, from twin JR), free axial length (m) for torsion, spiral cavity
# (r_in, r_out, mm) available in the CAD next to the joint, and where
JOINTS = [
    # pitch: bundle from the torso e-tray through the pitch shaft bore (Ø30) to the detach plug on the ferrule
    ("shoulder pitch", 260, 0.20, (22, 60), "inside the torso, around the pitch shaft between the bracket and the torso wall"),
    ("shoulder roll", 115, 0.10, (22, 50), "roll link, around the roll shaft between the walls and the clevis arms"),
    # loop 33: yaw limited to ±70° (reach 99 % and bimanual 4/4 unchanged; ±60° loses 3 of 4 bimanual tasks). The bundle
    # twists in torsion between a clamp at the roll-link top and one on the elbow-housing floor: free length ≈ 0.24 m
    ("upper-arm yaw", 140, 0.24, (0, 0), "torsion: clamp at the roll-link top and on the elbow-housing floor (yaw shaft + neck + housing run); no spiral (no room: belt between the yaw plates)"),
    ("elbow", 150, 0.13, (22, 50), "elbow housing, +Y side, between cassette plate B (y 26) and the +Y wall (y 53)"),
    ("wrist pitch", 220, 0.06, (16, 30), "wrist can end, around the DM-J4310 (Ø57 can: little room)"),
    ("wrist roll", 360, 0.03, (14, 24), "palm, around the STS3215 wrist-roll servo"),
]
SPIRAL_S = 0.60                    # spiral cable length per joint (m)


def bundle(beyond_wrist=False):
    """Cables crossing a joint. Past the wrist only 12 V/TTL (gripper servo) + USB (camera) + I²C (tactile, ToF) go on."""
    if beyond_wrist:                   # D12: the camera is MIPI-CSI on a 0.3 mm FFC through the wrist (bend r ~2 mm); bridge on the forearm
        return {"12 V + GND + TTL (servos, 24 AWG)": (3, 1.2), "I²C 4-core (tactile + ToF)": (4, 0.8), "CSI FFC 15-way (camera)": (1, 0.3)}
    return dict(CABLES)


def analyse():
    rows = []
    for name, rng, L, (ri, ro), where in JOINTS:
        b = bundle(beyond_wrist=name.startswith("wrist"))
        od_max = max(od for _, od in b.values())
        r_bend = BEND_K * od_max
        torsion_ok = (rng / 2) <= TORSION_DEG_PER_M * L
        ri_eff = max(ri, r_bend)
        cap = math.degrees(SPIRAL_S * (1 / (ri_eff / 1000) - 1 / (ro / 1000))) if ro > ri_eff else 0.0
        spiral_ok = cap >= rng * 1.2
        # for comparison: the original design, with the camera's stock Ø3.8 USB cable all the way to the hand
        od_nousb = max(max(od for _, od in b.values()), STOCK_USB_OD)
        ri2 = max(ri, BEND_K * od_nousb)
        cap2 = math.degrees(SPIRAL_S * (1 / (ri2 / 1000) - 1 / (ro / 1000))) if ro > ri2 else 0.0
        rows.append(dict(joint=name, range=rng, L=L, torsion_ok=torsion_ok, need_L=rng / 2 / TORSION_DEG_PER_M,
                         r_bend=r_bend, cavity=(ri, ro), spiral_cap=cap, spiral_ok=spiral_ok, cap_nousb=cap2, ok_nousb=cap2 >= rng * 1.2,
                         where=where, area=sum(n * math.pi * od ** 2 / 4 for n, od in b.values())))
    return rows


def main():
    rows = analyse()
    n_ok = sum(r["torsion_ok"] or r["spiral_ok"] for r in rows)
    n_ok2 = sum(r["torsion_ok"] or r["ok_nousb"] for r in rows)        # with the stock USB cable
    L = ["# Cable routing across the joints (generated by `python -m rnd.harness`)", "", __doc__.split("\n\n", 1)[1].strip(), "",
         f"**Joints the arm bundle can cross: {n_ok}/{len(rows)} with D12** (CSI flat-flex through the wrist + slim Ø2.2 USB from the forearm); "
         f"{n_ok2}/{len(rows)} with the camera's stock Ø3.8 USB cable.", "",
         "| Joint | Range | Torsion: free length / needed | Spiral cavity r_in–r_out (mm) | Bend r (10×OD) | Spiral capacity (S = 0.6 m) | Method (D12) | Stock USB |",
         "|---|---|---|---|---|---|---|---|"]
    for r in rows:
        m = "torsion ✅" if r["torsion_ok"] else ("clock spring ✅" if r["spiral_ok"] else "❌")
        L.append(f"| {r['joint']} | {r['range']}° | {r['L'] * 1000:.0f} / {r['need_L'] * 1000:.0f} mm {'✅' if r['torsion_ok'] else '❌'} | "
                 f"{r['cavity'][0]}–{r['cavity'][1]} | {r['r_bend']:.0f} mm | {r['spiral_cap']:.0f}° (need {r['range'] * 1.2:.0f}°) | {m} | "
                 f"{'✅' if r['torsion_ok'] or r['ok_nousb'] else '❌'} {r['cap_nousb']:.0f}° |")
    L += ["", "**Findings (loop 32):**",
          "- **No joint can rely on torsion through its bore.** The free axial lengths are 30–200 mm, and the ranges need 160–500 mm at ±360°/m. "
          "The old electrical note ('cables through the hollow shafts, no slip rings') needs **clock-spring spirals** at each joint.",
          "- **The camera's stock Ø3.8 USB cable** needs a 38 mm bend radius and fits through none of the wrist joints. **D12:**",
          "  - a MIPI-CSI camera on a thin FFC through both wrist joints",
          "  - a CSI→USB (UVC) bridge board on the forearm",
          "  - a slim 4-core high-flex USB cable (OD ≤ 2.2 mm) from there on",
          "- **Yaw (open):** the belt fills the space between the yaw plates, and there is no gap above or below them. Two fixes to evaluate against R1 reach:",
          "  - (a) a 12 mm spiral cassette under the yaw block (yaw_drop +12 mm)",
          "  - (b) limit yaw to about ±70° so torsion over the 0.2 m neck run suffices",          "",
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
