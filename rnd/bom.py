"""Bill of materials from the CAD quantities.  python -m rnd.bom

Prices: actuators and sensors from rnd/specs/*.yaml (sourced); hardware is ESTIMATE (typical online prices,
single quantity, 2026). The Pi 5 is excluded (the owner has one), as are tools.
"""
from __future__ import annotations

import json
from pathlib import Path

from .belts import STAGES, stage
from .budget import actuator_cost
from .params import BASELINE, act

DOCS = Path(__file__).resolve().parent.parent / "docs" / "rnd"


def items(d=BASELINE):
    names = [n for n, _ in d.arm.joints.values()] * 2 + [d.base.wheel_motor] * 4 + ["STS3215-C018"] * 2 + [d.upper.lift_motor]
    counts = {n: names.count(n) for n in sorted(set(names))}
    belts = [(j, round(stage(j, r, z, c, w)["length"] / 5) * 5) for j, r, z, c, w in STAGES]
    pp = json.loads((DOCS / "print_plan.json").read_text()) if (DOCS / "print_plan.json").exists() else {"kg": 10.4}
    I = []
    A = lambda cat, item, q, unit, src="ESTIMATE": I.append((cat, item, q, unit, src))
    for n, q in counts.items():
        A("Actuators", n, q, act(n)["price"], "actuators.yaml (mid list price)")
    A("Wheels & power", '6" DuraOmni omni wheel', 4, 33.0, "components.yaml")
    A("Wheels & power", "6S 10 Ah 21700 Li-ion pack, 35 A cont., 975 g (Lumenier NAV class)", 1, 150.0, "racedayquads.com listing (loop 13)")
    A("Wheels & power", "6S smart BMS, 40 A cont., UART/CAN telemetry + active balancing (JK-BMS class) — A13: telemetry for the dock stop, weekly 4.20 V balance charge", 1, 45.0, "ESTIMATE")
    A("Wheels & power", "6S charger, balance (ToolKitRC C6 class)", 1, 30.0, "racedayquads.com listing")
    A("Wheels & power", "Fuses + holders (40 / 20 / 15 / 15 / 10 / 7.5 / 5 A)", 1, 15.0)
    A("Wheels & power", "E-stop (A5 v2): 2 two-channel latching mushrooms + TE ALEV100 DC contactor (24 V coil, aux contact) + switched pre-charge (K_PRE relay + 3.3 Ω RH-50) + SN74LVC1G123 watchdog on its own LDO", 1, 150.0,
      "LEV100A5ANG $92 (Master Electronics); rest ESTIMATE")
    A("Wheels & power", "Regen clamp / bus discharge (A5 v2): 2 × Vishay RH-50 0.56 Ω + 2 × IRFB7530 + comparator/reference, gate driver, film caps, 85 °C thermal switch", 1, 58.0,
      "RH-50 ≈ $15 (RS 5-pack); rest ESTIMATE")
    A("Wheels & power", "DC-DC 24→12 V 10 A (head servos), 24→5 V 5 A", 1, 30.0)
    A("Wheels & power", "D14 elbow-node power: 24→12 V 3 A buck + half-duplex TTL transceiver (one per arm)", 2, 8.0)
    A("Wheels & power", "Connectors: XT60/XT30, 2 × 6-pin pogo blocks, JST-GH/VH, wire", 1, 50.0)
    A("Compute & sensors", "Raspberry Pi 5 8 GB", 1, 0.0, "owner has one")
    A("Compute & sensors", "Waveshare 2-CH CAN FD HAT for the Pi 5 (CAN B + C, mode A: separate SPI)", 1, 46.99, "waveshare.com (loop 14)")
    A("Compute & sensors", "USB-CAN adapter (A14: CAN A base + lift, and the second channel of each arm)", 3, 35.0, "components.yaml")
    A("Compute & sensors", "Bus-servo TTL adapter", 3, 8.0)
    A("Compute & sensors", "Powered USB3 hub", 1, 25.0)
    A("Compute & sensors", "Head RGB-D camera (Orbbec Gemini 2: 0.15–10 m, global-shutter IR, 98 g)", 1, 234.0, "store.orbbec.com (loop 14); Gemini 336 was $310")
    A("Compute & sensors", "Wrist camera (D12): global-shutter OV9281-class MIPI-CSI module + FFC through the wrist + CSI→USB (UVC) bridge on the forearm + slim 4-core high-flex USB (OD ≤ 2.2 mm)", 2, 55.0, "T01: USB versions $60–155; CSI module + bridge ESTIMATE")
    # --- sensor pack v2 (owner, loop 15: favour accuracy + dexterity over cost) ---
    A("Sensor pack v2", "Joint-side absolute encoders AS5048A + diametric magnets (8 arm belt joints + lift) → hand error 4.0 → 0.9 mm", 9, 12.0, "ESTIMATE")
    A("Sensor pack v2", "Sensor nodes: RP2040 + CAN transceiver (one per arm at the elbow, one in the base)", 3, 10.0, "ESTIMATE")
    A("Sensor pack v2", "Fingertip tactile pads: magnetised elastomer + MLX90393 magnetometer (AnySkin/ReSkin principle), 2 per gripper", 4, 10.0, "ESTIMATE")
    A("Sensor pack v2", "Gripper ToF proximity (VL53L4CD) for pre-grasp alignment", 2, 8.0, "ESTIMATE")
    A("Sensor pack v2", "Base cliff / stair-edge ToF sensors (VL53L1X, downward)", 4, 8.0, "ESTIMATE")
    A("Compute & sensors", "2D lidar (RPLIDAR C1)", 1, 75.0, "components.yaml")
    A("Compute & sensors", "IMU (BNO085) + optical-flow (PAA5100JE)", 1, 55.0, "components.yaml (flow)")
    A("Compute & sensors", "Mic array + speaker + chest/bumper LEDs (eyes are now drawn on the D11 screen)", 1, 36.0, "ESTIMATE")
    A("Compute & sensors", "D22 face screen: Waveshare ESP32-P4-WIFI6-Touch-LCD-7B (7\" 1024×600 cap. touch, CAN + RS485) + 24→5 V 2 A buck", 1, 46.99 + 6.0, "waveshare.com SKU 32510 (2026-11-27)")
    A("Drive & structure", "D23 consumer covers: soft-touch joint covers, TPU bumper skins, lift mast bellows/telescoping cover (filament + fasteners)", 1, 60.0, "ESTIMATE")
    A("Bearings", "6808-2RS (40×52×7), no-name 10-pack (check the 2RS variant)", 24, 2.02, "AliExpress MOCHU snapshot (loop 14); Zoro generic $4.71")
    A("Bearings", "6804-2RS (20×32×7): lift small hub + D16 wrist-roll spindles", 6, 0.92, "AliExpress MOCHU snapshot; Zoro $4.69")
    A("Bearings", "6801-2RS (12×21×5): D13 stage shafts, 2 per stage × 8 stages", 16, 0.9, "ESTIMATE")
    A("Bearings", "6805-2RS (25×37×7): head tilt passive pivot (A4)", 4, 1.15, "AliExpress snapshot; Zoro $5.69")
    A("Bearings", "U604ZZ grooved pulley bearings (A9 balancers: pulley A + tackle pulley)", 4, 1.5, "ESTIMATE")
    A("Bearings", "625ZZ (carriage, idlers)", 10, 0.25, "Walmart 40-pack")
    A("Bearings", "6708-2RS thin section (40×50×6), elbow belt cassettes (2 per arm)", 4, 3.0, "ESTIMATE")
    # --- self-docking charger (queue item 4, owner request; rnd/parts/dock.py, rnd/dock_sim.py) ---
    A("Docking charger", "6S Li-ion CC/CV charger 25.2 V, 4–5 A (≈ 2.5 h full charge)", 1, 35.0, "ESTIMATE")
    A("Docking charger", "Dock electronics: ESP32-C3 + MOSFET output switch + ID-pin handshake, fuse, DC jack, wiring", 1, 12.0, "ESTIMATE")
    A("Docking charger", "Contacts: 2 spring leaf contacts + 1 pogo pin (dock); 2 brass pads 60 × 12 mm + reverse-polarity diode/fuse (robot)", 1, 10.0, "ESTIMATE")
    A("Docking charger", "Floor screws + AprilTag print (dock)", 1, 2.0, "ESTIMATE")
    A("Drive & structure", "Torque hangers: 2 × M4×10 + 2 heat-set inserts each (wheels ×4 so far)", 4, 0.6, "ESTIMATE")
    A("Drive & structure", "Elbow belt cassette: 2 × M5×30 bolts + nylocs per arm; 4 × M3 heat-set inserts + screws for the bone cap", 2, 2.0, "ESTIMATE")
    # loop 40: 13 sockets (the wheels bolt on, D17/A11; the wrist roll lost its KX44, D16); D15 screwed sockets with 6 coil springs
    A("KX interfaces (13 sockets)", "G25 balls: 27 × Ø10 ($0.32), 6 × Ø8, 6 × Ø6", 1, 11.0, "Intech / Zoro listings")
    A("KX interfaces (13 sockets)", "Hardened dowel pins: 50 Ø5×14, 50 Ø4×12, 50 Ø3×8 (packs)", 1, 38.0, "Zoro listings")
    A("KX interfaces (13 sockets)", "D15 preload: 6 coil springs Ø4 × 8 per socket (assortment) + PTFE washer; 3 × M3 retainer screws + inserts", 13, 1.2, "ESTIMATE")
    A("KX interfaces (13 sockets)", "Torque pins Ø6/Ø5/Ø4 + magnet index", 13, 0.4)
    A("Drive & structure", "D13 small stages ×8: Ø12 h6 steel rod (2 × 300 mm), aluminium rigid flange couplings 12 mm, aluminium HTD 5M 15 mm pulleys 12 mm bore (6 × 20T, 2 × 18T)", 8, 12.0, "ESTIMATE")
    A("Drive & structure", "Wheel hubs bolted to the GIM output (D17/A11: no KX on the wheels): 4 × M4 per wheel", 4, 0.8, "ESTIMATE")
    A("Drive & structure", "Chassis ring (A11): 3 mm 6061 laser-cut ring tying the 4 deck quarters", 1, 40.0, "ESTIMATE (SendCutSend class)")
    A("Drive & structure", "Anti-tip casters (D18/A10): spring-loaded ball transfer units at the 4 diagonals", 4, 4.0, "ESTIMATE")
    A("Drive & structure", "Clock-spring ribbon stock (A3): 20 AWG silicone pairs, 26 AWG twisted pairs, slim USB, PET/Kapton carrier tape", 1, 30.0, "ESTIMATE")
    A("Drive & structure", "HTD 5M 15 mm closed belts (" + ", ".join(f"{j} {l} mm" for j, l in belts) + ") × 2 arms", 8, 9.0, "Bestorq 400-5M-15 $7.67 at MROSupply (loop 14); Gates ≈ $20–28")
    A("Drive & structure", "HTD 5M 15 mm open belt for the lift (2.4 m at $8.32/m) + 2 clamps", 1, 25.0, "Acorn listing")
    A("Drive & structure", "Al square tube 60×60×2, 1.1 m (mast), black anodised", 1, 45.0, "no US listing found: ESTIMATE")
    A("Drive & structure", "Carbon tubes Ø40×38 (0.5 m) + Ø30×28 (1 m), roll-wrapped", 1, 70.0, "eBay / Carbonwebshop listings")
    A("Drive & structure", '3" quick-release squeeze tri-clamp, 304 SS (arm detach; flange OD 90.9 mm confirmed)', 2, 25.0, "hosewarehouse (Dixon 13MHHM-Q300)")
    A("Drive & structure", "Lift lock (A15): bolted steel rack (module 1, 10 × 10, 500 mm) + hardened steel pawl + 12 V solenoid + springs; printed belt guards", 1, 35.0, "ESTIMATE")
    A("Drive & structure", "Balancers (A9): ISO 10243 green die spring Ø25/Ø12.5 × 115, 18.7 N/mm (Berger Tools V25-115 / Accu TJL25-115) + 2 mm UHMWPE cord", 2, 12.0, "ESTIMATE")
    A("Drive & structure", "Fasteners, heat-set inserts, epoxy, grease", 1, 60.0)
    A("Printed parts", f"PETG {pp['kg']:.1f} kg + 20 % spare (print_plan.md)", 1, round(pp["kg"] * 1.2 * 20, 0))
    A("Printed parts", "ASA 2 kg (A6: motor cans, KX hubs/collars and D13 hubs near motors)", 1, 40.0)
    return I


CONTINGENCY = 0.15      # A16 (review finding 18): price drift, shipping, tax


def report(d=BASELINE):
    I = items(d)
    sub = sum(q * u for _, _, q, u, _ in I)
    I.append(("Contingency (A16)", f"{CONTINGENCY:.0%} for price drift, shipping and tax", 1, round(sub * CONTINGENCY, 0), "review finding 18"))
    tot = sum(q * u for _, _, q, u, _ in I)
    cats = {}
    for c, _, q, u, _ in I:
        cats[c] = cats.get(c, 0) + q * u
    L = ["# Bill of materials v4 (auto-generated by `python -m rnd.bom`)", "",
         f"**Total ≈ ${tot:,.0f} landed** (${sub:,.0f} + {CONTINGENCY:.0%} contingency; target ≤ $4,000 since D9 (owner: sensors over savings); limit $5,000; excludes the owner's Pi 5 and tools). Single-unit online prices; hardware lines are ESTIMATEs.", "",
         "| Category | Subtotal |", "|---|---|"] + [f"| {c} | ${v:,.0f} |" for c, v in cats.items()] + [
         "", "| Category | Item | Qty | Unit $ | Line $ | Source |", "|---|---|---|---|---|---|"]
    L += [f"| {c} | {n} | {q} | {u:,.2f} | {q * u:,.0f} | {s} |" for c, n, q, u, s in I]
    return "\n".join(L), tot


if __name__ == "__main__":
    md, tot = report()
    (DOCS / "BOM_v4.md").write_text(md)
    (DOCS / "bom.json").write_text(json.dumps({"total": tot}))
    print(md[:1500])
