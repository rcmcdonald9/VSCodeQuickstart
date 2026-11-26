"""Milestone M1: one-arm bench kit (shoulder pitch module + roll/yaw + arm + hand on a desk clamp), priced from the BOM sources.
python -m rnd.m1_kit

Loop 40 refresh (owner: "buying the arm tomorrow, I need higher confidence first"): the list now follows the design as it
stands after the D13–D16 changes and review items A2–A9, A14, A16. Every line says which design element needs it.
"""
from __future__ import annotations

from pathlib import Path

from .params import BASELINE, act

DOCS = Path(__file__).resolve().parent.parent / "docs"
CONTINGENCY = 0.15            # A16: price drift, shipping and tax (review finding 18)

# (group, item, qty, unit $, why / where, status)  status: "verified" price from a retailer snapshot, "ESTIMATE" otherwise
def items(d=BASELINE):
    g, j, s = act("GIM6010-8"), act("DM-J4310-2EC_V1.1"), act("STS3215-C018")
    return [
        ("Actuators", "SteadyWin GIM6010-8 **24 V, SDC104 driver version**", 4, g["price"], "pitch, roll, yaw, elbow (whole-workspace margins 1.54–1.80×, rated/p95, 1 kg)", "verified"),
        ("Actuators", "Damiao DM-J4310-2EC **V1.1**", 1, j["price"], "wrist pitch (margin 1.74×)", "verified"),
        ("Actuators", "Feetech STS3215-C018 **12 V** (30 kg·cm)", 2, s["price"], "wrist roll, gripper", "verified"),
        ("Drive (D13)", "Steel rod Ø12 h6, 300 mm (cut to 4 × 70 mm stage shafts)", 1, 9.0, "D13 small stages (pitch, roll, yaw, elbow)", "ESTIMATE"),
        ("Drive (D13)", "Rigid flange coupling, 12 mm bore, aluminium", 4, 5.0, "D13: shaft → KX74 hub (4 × M4)", "ESTIMATE"),
        ("Drive (D13)", "Aluminium HTD 5M pulley, 15 mm belt, 12 mm bore: 3 × 20T + 1 × 18T", 4, 7.0, "D13 small pulleys", "ESTIMATE"),
        ("Drive", "HTD 5M-15 closed belts: 355, 475, 315, 295 mm (nearest stock to 354 / 475 / 313 / 294)", 4, 9.0, "pitch, roll (80T), yaw, elbow (rnd/belts.py)", "ESTIMATE"),
        ("Bearings", "61808 (40×52×7)", 8, 2.02, "pitch, roll, yaw, elbow shafts", "verified"),
        ("Bearings", "6708 (40×50×6) thin", 2, 2.5, "elbow belt cassette", "ESTIMATE"),
        ("Bearings", "6801 (12×21×5)", 8, 0.9, "D13 stage shafts (2 each)", "ESTIMATE"),
        ("Bearings", "61804 (20×32×7)", 2, 0.92, "D16 wrist-roll spindle", "verified"),
        ("Bearings", "U604ZZ grooved pulley bearing", 2, 1.5, "A9 balancer: pulley A + tackle pulley", "ESTIMATE"),
        ("Couplings (D15)", "KX hardware for 4 × KX74 + 1 × KX56 sockets: 30 hardened dowels, 15 G25 balls, 30 coil springs Ø4×8 (assortment), "
         "M3 screws + heat-set inserts, retaining compound", 1, 34.0, "D15 screwed sockets with 6 springs each", "ESTIMATE"),
        ("Couplings", "Torque pins (Ø6 / Ø5 / Ø4 dowels) + yaw stop pin", 1, 6.0, "cartridge torque reaction (21/21), yaw hard stops ±155°", "ESTIMATE"),
        ("Structure", "Carbon tubes Ø40×1 + Ø30×1 (one arm)", 1, 40.0, "upper-arm neck, forearm link", "ESTIMATE"),
        ("Structure", '3" quick-release tri-clamp (arm detach)', 1, 25.0, "KX-V ferrule at the shoulder", "ESTIMATE"),
        ("Balancer (A9)", "ISO 10243 light-load die spring, Ø25 hole, L0 ≈ 115 mm, rate ≈ 12 N/mm + 2 mm UHMWPE cord", 1, 14.0, "2:1 tackle balancer, K 4.5 N·m", "ESTIMATE"),
        ("Sensors", "4 × AS5048A joint encoders + 2 RP2040 CAN nodes (shoulder + elbow) + 2 tactile pads + palm ToF", 1, 4 * 12 + 20 + 2 * 10 + 8, "joint-side encoders (accuracy 0.9 mm)", "ESTIMATE"),
        ("Sensors (D14)", "Elbow node power + bus: 24→12 V **6 A** buck (two stalled STS3215 draw 5.4 A; Astra #16) + half-duplex TTL transceiver + 2 × MCP2562 CAN transceivers", 1, 14.0, "D14: 12 V and TTL made in the arm; CAN map sw/config/can_map.yaml", "ESTIMATE"),
        ("Sensors", "Global-shutter wrist camera (OV9281 MIPI-CSI) + CSI→USB UVC bridge + slim high-flex USB cable", 1, 70.0, "D12 wrist camera", "ESTIMATE"),
        ("Harness", "Clock-spring ribbon stock: 20 AWG silicone pair, 26 AWG twisted pairs, Kapton/PET carrier tape; JST-GH/VH kit", 1, 18.0, "6 joint crossings (rnd/harness.py)", "ESTIMATE"),
        ("Electronics (A14)", "USB-CAN adapter, candleLight / CANable 2.0 class", 2, 35.0, "A14: two CAN channels per arm (≈ 46 % load each at 500 Hz)", "verified"),
        ("Electronics", "USB TTL bus-servo adapter (bench debugging of the STS3215s)", 1, 8.0, "servo ID setup", "ESTIMATE"),
        ("Power (A5)", "24 V 15 A bench supply + 4,700 µF 35 V bus capacitor + inline fuse 15 A", 1, 55.0, "bench power", "ESTIMATE"),
        ("Power (A5)", "Regen clamp: 26.5 V shunt (comparator, IRFB4110-class MOSFET, 10 Ω 50 W Al-clad resistor, heatsink) + 33 V TVS", 1, 16.0,
         "**mandatory on a bench supply** (it can't sink regenerated current; GIM rated 28 V)", "ESTIMATE"),
        ("Power (A5)", "E-stop: 2-channel mushroom (2 NC) + 24 V contactor/relay (≥ 20 A) + watchdog relay board", 1, 30.0, "category-0 stop, SW2", "ESTIMATE"),
        ("Consumables", "Fasteners (M3/M4/M5), heat-set inserts, epoxy, grease, PTFE washers", 1, 25.0, "", "ESTIMATE"),
        ("Filament (A6)", "PETG 3 kg + ASA 1 kg (motor cans, KX hubs/collars, D13 hubs) + TPU 95A 0.25 kg", 1, 90.0, "A6: cans in ASA (can wall up to 63 °C)", "ESTIMATE"),
    ]


CHANGES = [
    "**Bearings:** 8 × 61804 + 4 × 6805 → 8 × 6801 (D13 stage shafts) + 2 × 61804 (D16 wrist-roll spindle) + 2 × 6708 (elbow cassette) + 2 × U604 (balancer).",
    "**D13 standard parts (new):** Ø12 steel shafts, 4 aluminium flange couplings, 4 aluminium HTD pulleys (were printed small hubs).",
    "**Roll belt:** 475 mm (80T roll pulley, loop 40); the others are unchanged.",
    "**KX hardware (D15):** coil springs (6 per socket) in place of a wave-spring stack, which never fitted; 5 sockets (the wrist roll lost its KX44, D16).",
    "**Balancer (A9):** a die spring through a 2:1 tackle in place of a 3 N/mm compression spring, which could not fit its cage.",
    "**Electronics:** regen clamp (A5, mandatory on a bench supply), 2 USB-CAN channels (A14), elbow-node buck + TTL (D14), 2-channel e-stop + watchdog.",
    "**Filament:** + 1 kg ASA for the motor cans and motor-side couplings (A6).",
    "**+15 % contingency** for price drift, shipping and tax (A16).",
]

# Order gates: what to confirm with the seller before paying (each one has bitten DIY builds)
CHECKS = [
    "GIM6010-8: the **24 V SDC104** driver version (some Amazon listings are 48 V with other torque figures); ask for the installation drawing (bolt circles are ESTIMATE; coupon C9 reads them).",
    "DM-J4310-2EC: **V1.1** (V1.2 listings mix in J4310P figures). Ask the seller for the output bearing's allowable radial load and moment (wrist pitch carries ≈ 0.9 N·m bending, dynamic).",
    "STS3215: the **12 V C018** (30 kg·cm) version, not the 7.4 V one.",
    "Aluminium HTD 5M pulleys: 15 mm belt width, **12 mm bore** (many default to 8 mm; bore-to-order is common), with two set screws.",
    "Belts: closed HTD **5M**, 15 mm wide, the four lengths above (± 5 mm is taken up by the eccentric bushings: ±1 mm centre ≈ ±2 mm length).",
]


def main():
    I = items()
    sub = sum(q * u for *_, q, u, _, _ in [(a, b, c, e, f, g) for a, b, c, e, f, g in I])
    tot = sub * (1 + CONTINGENCY)
    n_est = sum(1 for *_, st in I if st == "ESTIMATE")
    L = ["# Milestone M1: one-arm bench kit (generated by `python -m rnd.m1_kit`)", "",
         f"**≈ ${sub:,.0f} + {CONTINGENCY:.0%} contingency = ${tot:,.0f}** (loop 40 refresh for the buy decision). "
         f"{len(I) - n_est} lines are priced from retailer snapshots, {n_est} are ESTIMATE. Everything except the bench supply is reused in the full robot.", "",
         "**What it proves on real hardware:**",
         "- joint strength, belt behaviour and the D13 stages",
         "- the D15 KX sockets under real loads (slip, twist, re-seat repeatability)",
         "- the shoulder balancer (A9)",
         "- hand accuracy with the joint encoders (target ≤ 1 mm), and the compliance chain (rnd/stiffness.py)",
         "- the fin-ray grasp and the D16 wrist-roll spindle",
         "- motor and can temperatures (A6), CAN timing (SW1, A14), e-stop time (SW2)",
         "", "| Group | Item | Qty | Unit $ | Line $ | Why | Price |", "|---|---|---|---|---|---|---|"]
    L += [f"| {grp} | {n} | {q} | {u:,.2f} | {q * u:,.0f} | {why} | {st} |" for grp, n, q, u, why, st in I]
    L += [f"| | **Subtotal** | | | **{sub:,.0f}** | | |", f"| | Contingency {CONTINGENCY:.0%} (A16) | | | {sub * CONTINGENCY:,.0f} | | |",
          f"| | **Total** | | | **{tot:,.0f}** | | |", "",
          "## What changed since the loop-39 list", ""] + [f"- {c}" for c in CHANGES] + [
          "", "## Check with the seller before paying", ""] + [f"- {c}" for c in CHECKS] + [
          "", "**Order of work:** the coupon round C1 ([COUPONS.md](COUPONS.md)) can run in parallel once the P2S arrives; the actuators, bearings and "
          "electronics don't depend on any printed coupon result. Buying is the owner's decision."]
    (DOCS / "M1_BENCH_ARM.md").write_text("\n".join(L))
    print(L[2])
    return sub, tot


if __name__ == "__main__":
    main()
