"""Bill of materials from the CAD quantities.  python -m rnd.bom

Prices: actuators and sensors from rnd/specs/*.yaml (sourced); hardware is ESTIMATE (typical online prices,
single quantity, 2026). The Pi 5 is excluded (the owner has one), as are tools.
"""
from __future__ import annotations

import json
import math
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
    ns = 2 * len(STAGES)                 # belt stages on both arms (D24 inline roll: pitch, yaw, elbow; the roll has none)
    inline = d.arm.roll_drive == "inline"
    I = []
    A = lambda cat, item, q, unit, src="ESTIMATE": I.append((cat, item, q, unit, src))
    for n, q in counts.items():
        if n in BEST_PRICE:                                    # loop 47 cost pass: best verified price, same part
            A("Actuators", n, q, *BEST_PRICE[n])
        else:
            A("Actuators", n, q, act(n)["price"], "actuators.yaml (mid list price)")
    A("Wheels & power", '6" DuraOmni omni wheel', 4, 33.0, "components.yaml")
    A("Wheels & power", "6S 10 Ah 21700 Li-ion pack (Lumenier NAV, no BMS): 35 A cont. with an 80 °C cut-off, 90 A burst, 975 g; re-terminated XT90", 1, 150.0,
      "racedayquads.com listing (loop 13); ratings: RaceDayQuads / GetFPV (2026-11-27 search)")
    A("Wheels & power", "6S smart BMS JK-B2A8S20P-HC: 200 A cont. / 350 A peak, 2 A active balance, CAN + RS485, 2 NTC (Neon #2 F11: the 40 A class BMS tripped on the 74 A peak) "
      "— A13: telemetry for the dock stop and the software pack-current budget", 1, 95.0, "nkon.nl €83.95 (2026-11-27); 18650batterystore $145 (sold out)")
    A("Wheels & power", "Steel battery box for the base bay (pack + BMS, vented) — Neon #2 F11", 1, 20.0, "ESTIMATE")
    A("Wheels & power", "Fuses + holders: 60 A MIDI main (Neon #2 F1/F11: a 40 A main sat at 93 % of the 37 A sustained) + ATO 20 / 20 / 20 / 10 / 7.5 / 5 A", 1, 25.0,
      "ESTIMATE")
    A("Wheels & power", "E-stop (A5 v2): 2 two-channel latching mushrooms + TE ALEV100 DC contactor (24 V coil, aux contact) + switched pre-charge (K_PRE relay + 3.3 Ω RH-50) + SN74LVC1G123 watchdog on its own LDO", 1, 150.0,
      "LEV100A5ANG $92 (Master Electronics); rest ESTIMATE")
    A("Wheels & power", "Regen clamp / bus discharge (A5 v2, Neon #2 F1): 3 × Vishay **NH-50** 0.56 Ω (non-inductive) + 3 × VS-40CPQ060-N3 freewheel Schottky + 3 × IRFB7530 + "
      "TLV3202 / TL431B, gate driver + gate resistors, film caps, 85 °C thermal switch (robot peak regen 2.8 kW; electrical.md)", 1, 127.0,
      "ESTIMATE: NH-50 $30 each (IBS NH05050R00FE02 $15.01, Future NH05025R00FE02 $42.30); VS-40CPQ060-N3 RS £37 / 25; rest ESTIMATE")
    A("Wheels & power", "Safety-board logic (netlist v3, electrical.md): discharge one-shot (LVC1G123 on Ā), LVC1G74 latch + power-on reset, LVC1G11 / LVC1G32 / LVC1G04 (K1 hold on AUX), UCC27524, "
      "1 kΩ 3 W bleeder, 12 V buck + 3.3 V LDO, 2 × PC817, 2-layer PCB (the M1 bench board moves to the robot; its ideal diode stays on the bench)", 1, 40.0, "ESTIMATE")
    A("Drive & structure", "Thermal (Fable M7, thermal.md): aluminium finned rear caps on the 6 arm GIM6010 cartridges + 10 k NTC per GIM", 1, 6 * 15.0 + 6.0,
      "ESTIMATE (6061 CNC instant-quote class, $15 each)")
    A("Wheels & power", "DC-DC 24→12 V 10 A (head servos), 24→5 V 5 A", 1, 30.0)
    A("Wheels & power", "D14 elbow-node power: 24→12 V 6 A buck (two stalled STS3215 draw 5.4 A, Astra #16) + half-duplex TTL transceiver (one per arm)", 2, 10.0)
    A("Wheels & power", "Connectors (Neon #2 F10): XT90 (pack), 5 × XT60 (2 arm detach, 3 base ↔ upper), 2 × 10-pin + 1 × 12-pin pogo blocks (USB on the arm plugs), JST-GH/VH, wire", 1, 62.0)
    A("Wheels & power", "Lift umbilical (Neon #2 F10): drag chain 15 × 30 mm R 50, 0.55 m + 2 × 14 AWG and 3 × 18 AWG silicone pairs, CAN / e-stop / signal cores, shielded USB (1.2 m each)", 1, 30.0)
    A("Compute & sensors", "Raspberry Pi 5 8 GB", 1, 0.0, "owner has one")
    A("Compute & sensors", "USB-CAN adapter, candleLight/gs_usb (FYSETC UCAN, STM32F072): A14's five channels (CAN A base + lift, two per arm); replaces the Waveshare 2-CH HAT + 3 × $35 adapters", 5, 19.0,
      "MatterHackers FYSETC UCAN V1.0 $19 (2026-11)")
    A("Compute & sensors", "Bus-servo USB-TTL adapter (half-duplex, ID programming only: D14 elbow nodes and the torso node drive the TTL buses)", 1, 8.0)
    A("Compute & sensors", "Raspberry Pi 5 Active Cooler (official heatsink + PWM fan): keeps the control loop out of thermal throttling (Menlo sim2real lesson)", 1, 5.0)
    A("Compute & sensors", "Powered USB3 hub", 1, 25.0)
    A("Compute & sensors", "Head RGB-D camera (Orbbec Gemini 2: 0.15–10 m, global-shutter IR, 98 g)", 1, 234.0, "store.orbbec.com (loop 14); Gemini 336 was $310")
    A("Compute & sensors", "Wrist camera (D12): global-shutter OV9281-class MIPI-CSI module + FFC through the wrist + CSI→USB (UVC) bridge on the forearm + slim 4-core high-flex USB (OD ≤ 2.2 mm)", 2, 55.0, "T01: USB versions $60–155; CSI module + bridge ESTIMATE")
    # --- sensor pack v2 (owner, loop 15: favour accuracy + dexterity over cost) ---
    A("Sensor pack v2", "Joint-side absolute encoders AS5048A + diametric magnets (8 arm belt joints + lift) → hand error 4.0 → 0.9 mm", 9, 12.0, "ESTIMATE")
    A("Sensor pack v2", "Sensor nodes: RP2040 + CAN transceiver (shoulder + elbow node per arm, sw/config/can_map.yaml 0x30 / 0x31; one in the base)", 5, 10.0, "ESTIMATE")
    A("Sensor pack v2", "Fingertip tactile pads: magnetised elastomer + MLX90393 magnetometer (AnySkin/ReSkin principle), 2 per gripper", 4, 10.0, "ESTIMATE")
    A("Sensor pack v2", "Gripper ToF proximity (VL53L4CD) for pre-grasp alignment", 2, 8.0, "ESTIMATE")
    A("Sensor pack v2", "Base cliff / stair-edge ToF sensors (VL53L1X, downward)", 4, 8.0, "ESTIMATE")
    A("Compute & sensors", "2D lidar (RPLIDAR C1)", 1, 69.0, "RobotShop US $69 (2026-11); Waveshare $79.99")
    A("Compute & sensors", "IMU (BNO085) + optical-flow (PAA5100JE)", 1, 55.0, "components.yaml (flow)")
    A("Compute & sensors", "Mic array + speaker + chest/bumper LEDs (eyes are now drawn on the D11 screen)", 1, 36.0, "ESTIMATE")
    A("Compute & sensors", "D22 face screen: Waveshare ESP32-P4-WIFI6-Touch-LCD-7B (7\" 1024×600 cap. touch, CAN + RS485) + 24→5 V 2 A buck", 1, 46.99 + 6.0, "waveshare.com SKU 32510 (2026-11-27)")
    A("Drive & structure", f"D23 consumer covers: TPU 95A gaiters/bumper skins (1 kg) + fasteners; the rigid PETG covers ({COVERS_KG} kg, covers.printed()) are in the PETG bundles below", 1, 30.0,
      "ESTIMATE (TPU 95A ≈ $20–25 / kg)")
    A("Bearings", "6808-2RS (40×52×7), no-name 10-pack (check the 2RS variant): 2 per wheel + 2 per arm belt stage", 8 + 2 * ns, 2.02, "AliExpress MOCHU snapshot (loop 14); Zoro generic $4.71")
    A("Bearings", "6804-2RS (20×32×7): lift small hub + D16 wrist-roll spindles", 6, 0.92, "AliExpress MOCHU snapshot; Zoro $4.69")
    A("Bearings", f"6801-2RS (12×21×5): D13 stage shafts, 2 per stage × {ns} stages", 2 * ns, 0.9, "ESTIMATE")
    A("Bearings", "6805-2RS (25×37×7): head tilt passive pivot (A4)", 4, 1.15, "AliExpress snapshot; Zoro $5.69")
    A("Bearings", "U604ZZ grooved pulley bearings (A9 balancers: pulley A + tackle pulley)", 4, 1.5, "ESTIMATE")
    A("Bearings", "625ZZ (carriage, idlers)", 10, 0.25, "Walmart 40-pack")
    A("Bearings", "6708-2RS thin section (40×50×6), elbow belt cassettes (2 per arm)", 4, 3.0, "ESTIMATE")
    # --- self-docking charger (queue item 4, owner request; rnd/parts/dock.py, rnd/dock_sim.py) ---
    A("Docking charger", "6S Li-ion CC/CV charger 25.2 V, 4–5 A (≈ 2.5 h full charge)", 1, 35.0, "ESTIMATE")
    A("Docking charger", "Dock electronics: ESP32-C3 + MOSFET output switch + ID-pin handshake, fuse, DC jack, wiring", 1, 12.0, "ESTIMATE")
    A("Docking charger", "Contacts (Neon #2 F12): 2 spring leaf contacts + 1 pogo pin (dock); 3 brass pads 60 × 28 mm + reverse-polarity diode/fuse + ID-line 1 kΩ / 5.6 V TVS (robot)", 1, 14.0, "ESTIMATE")
    A("Docking charger", "Dock baseplate held down by the robot's wheels (no floor screws; dock_sim.md): 6 mm sheet with two wheel cradles + rubber feet + AprilTag print", 1, 12.0, "ESTIMATE")
    A("Drive & structure", "Torque hangers: 2 × M4×10 + 2 heat-set inserts each (wheels ×4 so far)", 4, 0.6, "ESTIMATE")
    A("Drive & structure", "Elbow belt cassette: 2 × M5×30 bolts + nylocs per arm; 4 × M3 heat-set inserts + screws for the bone cap", 2, 2.0, "ESTIMATE")
    # loop 40: 13 sockets (the wheels bolt on, D17/A11; the wrist roll lost its KX44, D16); D15 screwed sockets with 6 coil springs
    # wave 1 (D24): the 8 arm cartridges + 2 wrist DMs bolt in, so only the lift (KX74) and head pan + tilt (KX44) keep a socket
    A("KX interfaces (3 sockets)", "G25 balls: 3 × Ø10, 6 × Ø6", 1, 3.0, "Intech / Zoro listings, pro rata")
    A("KX interfaces (3 sockets)", "Hardened dowel pins: 50 Ø5×14, 50 Ø3×8 (packs; also the yaw stop pins and ferrule dowels need Ø6, below)", 1, 25.0, "Zoro listings (2 of the 3 packs)")
    A("KX interfaces (3 sockets)", "D15 preload: 6 coil springs Ø4 × 8 per socket (assortment) + PTFE washer; 3 × M3 retainer screws + inserts", 3, 1.2, "ESTIMATE")
    A("KX interfaces (3 sockets)", "Torque pins Ø6/Ø5/Ø4 + magnet index (KX cartridges: 4 wheels, lift, head ×2)", 7, 0.4)
    A("Drive & structure", "Dowels Ø6 × 10–12: yaw hard-stop pins + KX-V ferrule dowels (3 per arm)", 1, 6.0, "ESTIMATE")
    A("Drive & structure", f"D13 v3 small stages ×{ns}, keyed (belts.md: a plain Ø12 clamp hub is 0.65× at the 11 N·m peak): Ø12 h6 rod with a 4 × 2.5 keyway (share of 2 × 300 mm), "
      "aluminium HTD 5M 15 mm 20T pulley 12 H7 + keyway, DIN 6885 4 × 4 keys", ns, 19.5, "ESTIMATE (m1_kit KEYED_* lines)")
    A("Drive & structure", "D24 Oldham couplings: **D50**, 12 H7 clamp hub + 4 mm keyway, acetal disc (one per arm belt stage; the D40 disc was 0.86×, belts.md; the motor-side tongue adapter is printed)", ns, 20.0,
      "ESTIMATE generic D50 (m1_kit OLDHAM); Ruland MOCT41-12-A (D41) was $55.20 at Motion")
    A("Drive & structure", "D24/D19 heat-set inserts: 2 × 100 M4 short (≈ 51 per arm: flanges 6 each, seat caps, yaw flange + plate, outer wall, pitch hub) + 100 M3 (≈ 45 per arm: cap rims, forearm rim, palm, bone cap)", 1, 2 * 15.93 + 12.0,
      "Newark M4 brass $15.93 / 100; M3 ESTIMATE")
    A("Drive & structure", "D24 flange screws: 70 × M4 × 12 countersunk ISO 10642 (6 per bolted cartridge × 10 + spares)", 1, 10.0, "ESTIMATE")
    A("Drive & structure", "Wheel hubs bolted to the GIM output (D17/A11: no KX on the wheels): 4 × M4 per wheel", 4, 0.8, "ESTIMATE")
    A("Drive & structure", "Chassis ring (A11): 3 mm 6061 laser-cut ring tying the 4 deck quarters", 1, 40.0, "ESTIMATE (SendCutSend class)")
    A("Drive & structure", "Anti-tip casters (D18/A10): spring-loaded ball transfer units at the 4 diagonals", 4, 4.0, "ESTIMATE")
    A("Drive & structure", "Clock-spring ribbon stock (A3, Neon #2 F10): 16 AWG high-flex ETFE/FEP pairs (was 20 AWG silicone), 26 AWG twisted pairs, shielded slim USB, PET/Kapton carrier + PTFE tape liners", 1, 40.0, "ESTIMATE")
    A("Drive & structure", "HTD 5M 15 mm closed belts (" + ", ".join(f"{j} {l} mm" for j, l in belts) + ") × 2 arms", 2 * len(belts), 9.0, "Bestorq 400-5M-15 $7.67 at MROSupply (loop 14); Gates ≈ $20–28")
    A("Drive & structure", "HTD 5M 15 mm open belt for the lift (2.4 m at $8.32/m) + 2 clamps", 1, 25.0, "Acorn listing")
    if inline:                                                  # D24 inline roll (rnd/parts/roll.py INLINE section), per arm
        A("Bearings", "61806-2RS (30×42×7): inline roll rear support on the roll link's stub", 2, 3.0, "ESTIMATE (m1_kit Hold B)")
        A("Drive & structure", "Inline roll output adapter: 7075-T6 Ø76 × 7, Ø30 pilot, 3 × Ø4 H7 reamed + 3 × M4 csk, 8 × M5 tapped on PCD 64 (CNC, qty 2)", 2, 30.0,
          "ESTIMATE: JLCCNC / PCBWay-class instant quote for a 3-axis Ø76 × 7 part ≈ $20–25 each at qty 2 + shipping share; a printed adapter fails (3 × M4 into PETG ≈ 85 N·m < 103 N·m stall, roll.py)")
        A("Drive & structure", "Inline roll hardware: Ø80/Ø52 × 2 steel clamp ring (laser-cut, rides on the chassis-ring order), 8 × M5×16 DIN 7984 10.9, 8 × M4×18 ISO 4762, "
          "3 × M4×10 ISO 10642, 3 × Ø4 m6 dowels, end washer + M5", 2, 11.0, "ESTIMATE (ring ≈ $5 SendCutSend class; screws/dowels ≈ $6)")
    A("Drive & structure", "Al square tube 60×60×2, 1.1 m (mast), black anodised", 1, 45.0, "no US listing found: ESTIMATE")
    A("Drive & structure", "Carbon tubes Ø40×38 (0.5 m) + Ø30×28 (1 m), roll-wrapped", 1, 70.0, "eBay / Carbonwebshop listings")
    A("Drive & structure", '3" quick-release squeeze tri-clamp, 304 SS (arm detach; flange OD 90.9 mm confirmed)', 2, 25.0, "hosewarehouse (Dixon 13MHHM-Q300)")
    A("Drive & structure", "Lift lock (A15): bolted steel rack (module 1, 10 × 10, 500 mm) + hardened steel pawl + 12 V solenoid + springs; printed belt guards", 1, 35.0, "ESTIMATE")
    A("Drive & structure", "Balancers (A9): ISO 10243 green die spring Ø25/Ø12.5 × 115, 18.7 N/mm (Berger Tools V25-115 / Accu TJL25-115) + 2 mm UHMWPE cord", 2, 12.0, "ESTIMATE")
    A("Drive & structure", "Fasteners (other), heat-set inserts (base, torso, head), epoxy, grease", 1, 60.0)
    kg = pp["kg"] + COVERS_KG                                   # print plan + the D23 rigid covers (not in print_plan.md)
    n10 = math.ceil(kg * 1.2 / 10)                              # loop 47: 10 kg bundles instead of $20/kg single spools
    A("Printed parts", f"PETG {pp['kg']:.1f} kg (print_plan.md) + {COVERS_KG} kg D23 covers + 20 % spare = {kg * 1.2:.1f} kg: {n10} × Elegoo PETG Pro 10 kg bundle", n10, 109.99,
      "us.elegoo.com PETG Pro 10 kg $109.99 (2026-11; list $140)")
    A("Printed parts", "ASA 2 kg (A6: motor cans, KX hubs/collars and D13 hubs near motors)", 1, 40.0)
    return I


# loop 47 cost pass: the same parts at the best verified price (motor choices unchanged); wave-2 cost pass (2026-11-27) below
BEST_PRICE = {"GIM6010-8": (90.0, "aifitlab $90 with SDC104 driver (actuators.yaml, loop 14); SteadyWin store $93.88 with GDS68"),
              "GIM8108-36": (240.0, "with a driver: SteadyWin store $239.80 (GDM810 = SDC303, 24 V), aifitlab $240 (GDK810); bare motor $190 "
                                    "(aifitlab, backordered), so the yaml mid $171 had no driver (2026-11-27 search snapshots)"),
              "DM-J4310-2EC_V1.1": (125.0, "AIFit Lab $125 (24 V V1.1/V3, US seller; backordered 2026-11-27): the full-robot buy "
                                           "comes after M1; M1 keeps Robotics Center $150 in stock")}
COVERS_KG = 1.4         # D23 rigid PETG covers, both arms: 1,126 cm³ (rnd/parts/covers.py printed(), arm pieces ×2) × 1.27 g/cm³

CONTINGENCY = 0.15      # A16 (review finding 18): price drift, shipping, tax
WAVE1_BEFORE = 4898.0   # landed total after the loop-47 cost pass (bom.json before wave 1)
WAVE1_AFTER = 5013.0    # after wave 1 (RND_LOG, buy-list track)
WAVE2_BEFORE = 5199.0   # after the owner's inline-roll OK, before the wave-2 cost pass
WAVE2_AFTER = 5308.0    # after the wave-2 cost pass (before the Fable-review fixes)
NEON2_BEFORE = 5485.0   # bom.json before the Neon review #2 electrical / harness / dock / keyed-drive fixes (2026-11-27)
HOLDS = {}


def _sd(x):
    return f"{'+' if x >= 0 else '−'}${abs(x):,.0f}"


def holds():
    """Robot-wide deltas (with contingency) of the M1 kit's two holds (rnd/m1_kit.py prices), both arms."""
    from . import m1_kit as k
    c = 1 + CONTINGENCY
    a = k.line_sum
    return dict(d20=2 * (a(k.HOLD_A["yes"]) - a(k.HOLD_A["no"])) * c, d20_hi=2 * k.HOLD_A["hi"]() * c,
                inline=2 * (a(k.HOLD_B["yes"]) - a(k.HOLD_B["no"])) * c, inline_hi=2 * k.HOLD_B["hi"]() * c)


def report(d=BASELINE):
    HOLDS.update(holds())
    pp = json.loads((DOCS / "print_plan.json").read_text()) if (DOCS / "print_plan.json").exists() else {"kg": 10.4}
    I = items(d)
    sub = sum(q * u for _, _, q, u, _ in I)
    I.append(("Contingency (A16)", f"{CONTINGENCY:.0%} for price drift, shipping and tax", 1, round(sub * CONTINGENCY, 0), "review finding 18"))
    tot = sum(q * u for _, _, q, u, _ in I)
    cats = {}
    for c, _, q, u, _ in I:
        cats[c] = cats.get(c, 0) + q * u
    L = ["# Bill of materials v4 (auto-generated by `python -m rnd.bom`)", "",
         f"**Total ≈ ${tot:,.0f} landed** (${sub:,.0f} + {CONTINGENCY:.0%} contingency; target ≤ $4,000 since D9 (owner: sensors over savings); limit $5,500 (owner, 2026-11-27; was $5,000); excludes the owner's Pi 5 and tools). Single-unit online prices; hardware lines are ESTIMATEs.", "",
         "**Loop 47 cost pass** (motor choices unchanged; before → after, each with the 15 % contingency):",
         "- A5 v2 safety electrics added first: DC contactor with aux contact, switched pre-charge, energy-rated regen clamp: $71 → $208 of lines (+$158 landed): $5,030 → $5,188.",
         "- Savings (−$290 landed): PETG in 10 kg bundles ($333 → $220); five $19 candleLight USB-CAN adapters in place of the CAN HAT + three $35 adapters "
         "($152 → $95); one USB-TTL adapter instead of three (the D14 nodes drive the buses); no separate balance charger (the dock's CC/CV charger + the BMS's active "
         "balancing do the weekly 4.20 V charge, A13); GIM6010-8 at the verified $90, DM-J4310 at $150 in stock, RPLIDAR C1 at $69.",
         "- Result: **$4,898**, under the $5,000 limit; the $4,000 target would need the motor or sensor choices reopened.", "",
         f"**Wave 1 (D24 bolted cartridges, D19 one-piece shafts; 2026-11-27): ${WAVE1_BEFORE:,.0f} → ${WAVE1_AFTER:,.0f} landed.** The arm's 10 KX sockets go "
         "(balls, dowel packs, springs, torque pins: −$35 of lines); in: 8 Oldham couplings ($96; the spare acetal rod is an M1 bench item), heat-set inserts ($44), flange "
         "screws ($10), Ø6 dowels for the yaw stops and ferrules ($6); the D13 flange couplings go (−$24); D14 buck 3 → 6 A (+$4); PETG from the "
         "re-sliced print plan. +$13 against the $5,000 limit at the time. The owner's OK for the inline GIM8108-36 roll then put two GIM8108-36 "
         f"in place of two GIM6010-8 (at the yaml's $171): ${WAVE2_BEFORE:,.0f}.", "",
         f"**Fable review fixes (2026-11-27, M2 + M7): ${WAVE2_AFTER:,.0f} → ${tot:,.0f} landed ({_sd(tot - WAVE2_AFTER)}).** A third clamp resistor + MOSFET "
         "(the inline roll raised the robot's peak regen to 2.8 kW), the safety-board logic the netlist needs (one-shot discharge, latch, comparators), and "
         "aluminium finned rear caps + thermistors on the six arm GIM6010 cartridges (a closed ASA sleeve allows only 0.65 × rated continuous). The C1/M1 "
         "joint hardware (tie-rods, 6061 yaw flange, 7075 crank, steel stop pins) is in the M1 kit and is added here by its own track.", "",
         f"**Neon review #2 fixes (2026-11-27; findings 1, 10, 11, 12 and the keyed D50 drive of 2/15): ${NEON2_BEFORE:,.0f} → ${tot:,.0f} landed ({_sd(tot - NEON2_BEFORE)}).** "
         "NH-50 clamp resistors + freewheel Schottkys (+$51), a 200 A BMS for the 40 A class (+$50), a steel battery box (+$20), 60 A main fuse (+$10), XT90 / XT60 / USB pogo "
         "connectors (+$12), the lift umbilical (+$30), 16 AWG fluoropolymer ribbon wire (+$10), the dock baseplate and taller pads instead of floor screws (+$14), "
         "keyed stages and D50 Oldhams instead of plain shafts and D40s (+$111), the netlist-v3 logic (+$5); plus contingency.", "",
         f"**Wave-2 cost pass (2026-11-27; motor choices and safety items unchanged): ${WAVE2_BEFORE:,.0f} → ${WAVE2_AFTER:,.0f} landed ({_sd(WAVE2_AFTER - WAVE2_BEFORE)}).** "
         "The inline roll was carried as a bare motor with the belt-roll parts still in the list, so corrections come first (lines, before contingency):",
         "- GIM8108-36 $171 → **$240** with its 24 V driver (SteadyWin store $239.80 GDM810 = SDC303; aifitlab $240 GDK810; $190 is the bare motor): +$138.",
         "- Belt-roll parts out (2 Oldham, 2 D13 stages, 2 belts, 4 × 61808, 4 × 6801: −$72); inline parts in (2 × 7075 output adapter at a $30 CNC "
         "ESTIMATE, 2 × 61806, steel clamp ring + screws + dowels: +$88). A printed adapter is not an option (3 × M4 into PETG ≈ 85 N·m < 103 N·m stall).",
         "- Sensor nodes 3 → 5 (sw/config/can_map.yaml has a shoulder and an elbow node per arm): +$20.",
         "- Savings: DM-J4310 $150 → **$125** (AIFit Lab, 24 V, US seller, backordered: the full-robot buy comes after M1): −$50; D23 covers $60 → $30 "
         f"(the rigid covers, {COVERS_KG} kg of PETG, fit in the 10 kg bundles: 18.2 kg needed with spare of 20 kg bought; TPU 95A + fasteners stay): −$30.",
         "- Not taken: Foxtech DM-J4310 $116–120 (ships from China: tariff/duty exposure eats the gap); pultruded CF (higher axial E, but the forearm's "
         "double-D pinch clamp and 1 mm wall split a unidirectional tube; roll-wrapped listings are already $63–70); DIY 6S2P from cells (≈ −$50, but a "
         "home-welded pack without a spot welder or cell matching is a safety risk in a home robot); generic D40 Oldham (no sourced price under $12; Ruland/GMT $36–67); cheaper "
         "heat-set inserts (the short M4 is $11–21 / 50 elsewhere); a lower contingency (this pass found a +40 % price on one line, and China-direct "
         "parts now pay duty: 15 % stays); Gemini 2 / RPLIDAR C1 / battery / GIM6010-8 are already at the best sourced price.",
         f"- **{_sd(tot - 5500)} against the $5,500 limit** (owner raised it from $5,000 on 2026-11-27 to keep the inline roll and the review fixes). The belt roll (fallback `PIP_ROLL_DRIVE=belt`) is "
         f"{_sd(-HOLDS['inline'])} landed; the owner chose the inline roll for its hand-feel and slimmer shoulder.", "",
         "**Pending decisions (not in the total; the M1 kit's holds, robot-wide, with contingency):**",
         f"- **D20 GIM8108-8 at shoulder pitch + elbow** (waits on the seller's 24 V answer): 4 × GIM6010-8 → 4 × GIM8108-8: "
         f"**{_sd(HOLDS['d20'])}** (up to {_sd(HOLDS['d20_hi'])} at the dearest listing) → ${tot + HOLDS['d20']:,.0f}.",
         f"- **D24 inline GIM8108-36 at shoulder roll: in the total** (owner OK 2026-11-27). Against the belt roll it costs "
         f"**{_sd(HOLDS['inline'])}** (up to {_sd(HOLDS['inline_hi'])} at the dearest listing).",
         "- Prices: [M1_BENCH_ARM.md](../M1_BENCH_ARM.md) (GIM8108-8 ≈ $115 ESTIMATE; GIM8108-36 $240 with the SDC303 driver).", "",
         "| Category | Subtotal |", "|---|---|"] + [f"| {c} | ${v:,.0f} |" for c, v in cats.items()] + [
         "", "| Category | Item | Qty | Unit $ | Line $ | Source |", "|---|---|---|---|---|---|"]
    L += [f"| {c} | {n} | {q} | {u:,.2f} | {q * u:,.0f} | {s} |" for c, n, q, u, s in I]
    return "\n".join(L), tot


if __name__ == "__main__":
    md, tot = report()
    (DOCS / "BOM_v4.md").write_text(md)
    (DOCS / "bom.json").write_text(json.dumps({"total": tot, "with_d20": tot + HOLDS["d20"], "with_d20_inline": tot + HOLDS["d20"]}))
    print(md[:1500])
