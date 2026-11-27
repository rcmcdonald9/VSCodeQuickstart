"""Milestone M1: one-arm bench kit, bought in two steps (Fable review 2026-11-27: buy with changes).
python -m rnd.m1_kit

- **Step 1: prove the GIM protocol and the safety chain on ONE joint.** One GIM6010-8 (the yaw motor later), the DM-J4310,
  the two STS3215, the CAN adapters, the bench supply and the whole safety board, the cheap drive parts of one D13 stage on
  a test lever, thermistors and one aluminium rear cap, inserts and screws for the coupons, filament. Nothing in it waits
  on a seller answer or a CAD fix, and every part is reused in the arm (except the bench tools).
- **Step 2: the rest of the arm**, bought only when Step 1's exit criteria are met (docs/M1_READINESS.md): the common arm
  parts, plus two holds that each list what they buy if the answer is yes and what they buy instead if no:
  - **Hold A (D20)**: GIM8108-8 at shoulder pitch + elbow, waiting on the seller's 24 V confirmation; **not motors only**
    (Fable M6): Ø50 Oldham couplings, a Ø15 keyed stage, 25 mm belts and bearings for 22 N·m at the 20T pulley.
  - **Hold B (D24, owner-approved 2026-11-27)**: the inline GIM8108-36 roll (the plan), the belt roll as the fallback.
The old "zero-regret" advice (buy the three fallback GIM6010s now) is dropped: no GIM can be commanded until its CAN
protocol is known, and the GIM8108-36 has a different driver (docs/sw/gim_protocol_notes.md).
Every price is a retailer snapshot (source named) or ESTIMATE.
"""
from __future__ import annotations

from pathlib import Path

DOCS = Path(__file__).resolve().parent.parent / "docs"
CONTINGENCY = 0.15            # A16: price drift, shipping and tax (review finding 18)
E = "ESTIMATE"
E_D20 = "ESTIMATE (D20 redesign: the cartridge track sets the exact parts)"

# --- sourced unit prices used in more than one list (search snapshots 2026-11-27 unless noted) -------------------------
GIM6010 = (90.0, "aifitlab $90 with SDC104 (loop 14); Robotics Center lists $70, backordered (2026-11-27)")
GIM8108_8 = (115.0, "ESTIMATE: aifitlab $85 bare (no driver, backordered) + ≈ $25 driver (the GIM6010 gap $70 → $90); "
                    "openelab.com $113.99–213.99 by variant")
GIM8108_8_HI = 213.99
GIM8108_36 = (240.0, "SteadyWin store $239.80 with the GDM810 (SDC303, 24 V) driver; aifitlab $240 with GDK810 ($190 bare, backordered); "
                     "openelab.com $303.99+ with a driver (2026-11-27 search snapshots)")
GIM8108_36_HI = 306.99
BELT = (9.0, "Bestorq 400-5M-15 $7.67 at MROSupply (loop 14); Gates ≈ $20–28")
AL_PULLEY = (7.0, E)
OLDHAM = (12.0, "ESTIMATE generic D40 3-piece; Ruland MOCT41-12-A (Ø41.3, fits the Ø43 nose bore) $55.20 at Motion")
B61808 = (2.02, "AliExpress MOCHU 10-pack snapshot (loop 14)")
B6801 = (0.90, E)
PSU = (174.80, "Bravo Electro $174.80; Master Electronics $170.13 (2026-11-27 search snapshots)")
AL_CAP = (15.0, "ESTIMATE: 6061 Ø86–96 × 6 disc with fins, JLCCNC / PCBWay-class instant quote")


# (group, item, qty, unit $, why / where, source)
def step1():
    """Step 1: one joint, the GIM protocol and the safety chain (Fable verdict)."""
    j, s = (150.0, "Robotics Center $150 in stock (2026-11-27); AIFIT $125 backordered"), (19.5, "loop 14 retailer snapshot")
    return [
        ("Actuators", "SteadyWin GIM6010-8 **24 V, SDC104 driver version** (ONE)", 1, *GIM6010[:1],
         "the Step-1 joint (protocol discovery, limits, CAN timeout, e-stop on a real GIM); later the upper-arm yaw (20T:40T)", GIM6010[1]),
        ("Actuators", "Damiao DM-J4310-2EC **V1.1**", 1, *j[:1], "wrist pitch; its MIT protocol is known: the second motor on the Step-1 bus", j[1]),
        ("Actuators", "Feetech STS3215-C018 **12 V** (30 kg·cm)", 2, *s[:1], "wrist roll (D16 spindle), gripper", s[1]),
        ("Electronics (A14)", "USB-CAN adapter, candleLight / gs_usb (FYSETC UCAN, STM32F072), onboard 120 Ω (jumper)", 2, 19.0,
         "can_map.yaml buses B1 + B2 = SocketCAN can0 / can1", "MatterHackers $19 (2026-11)"),
        ("Electronics", "USB TTL bus-servo adapter (STS3215 ID setup; drives the servos in Step 1)", 1, 8.0, "servo IDs 1 / 2", E),
        ("Electronics", "Raspberry Pi 5 Active Cooler (official; heatsink + PWM fan)", 1, 5.0, "no thermal throttling under the 500 Hz loop (Menlo sim2real lesson: throttling quietly degraded actuator response)", "raspberrypi.com list price $5 (ESTIMATE until a retailer snapshot)"),
        ("Power (M2)", "**Mean Well SE-1000-24** (24 V 41.7 A, 1 kW), set 24.0–25.2 V", 1, *PSU[:1],
         "arm 14.8 A sustained / 31.5 A peak at the rated drive limit (electrical.md *Bench supply*); the old 10 A supply was a third of the peak", PSU[1]),
        ("Power (A5)", "Adjustable 0–30 V / 10 A CC/CV bench supply", 1, 65.71, "the 1 A current-limited first power-up only (drives disabled)",
         "Newegg generic 30 V 10 A CC/CV $65.71 (2026-11-27)"),
        ("Power (M2)", "4,700 µF 35 V bus capacitor + 40 A (main) and 20 A slow-blow (arm lead) ATO fuses with holders + 10 / 14 AWG silicone wire, XT60 / XT30",
         1, 22.0, "electrical.md safety-board netlist F1, F3, C1", E),
        ("Power (A5 v2)", "Regen clamp / bus discharge: **3** × Vishay RH-50 0.56 Ω + 3 × IRFB7530 + gate driver, film caps, 85 °C thermal switch, heatsink plate",
         1, 76.0, "mandatory on a bench supply; 3rd resistor since the inline roll (robot peak regen 2.8 kW; electrical.md)", "RH-50 ≈ $15 (RS 5-pack); rest ESTIMATE"),
        ("Power (A5 v2)", "E-stop: 2 two-channel latching mushrooms + TE ALEV100 DC contactor (24 V coil, aux contact) + K_PRE relay + 3.3 Ω RH-50 pre-charge "
         "+ SN74LVC1G123 watchdog on its own LDO", 1, 150.0, "category-0 stop; watchdog 25 ms (sized on sw/loop_jitter.py)",
         "LEV100A5ANG $92 (Master Electronics); rest ESTIMATE"),
        ("Power (M2)", "Safety-board logic (netlist in electrical.md): LM74700-Q1 + IRFB7530 ideal diode, 2nd LVC1G123 (discharge one-shot), LVC1G74 latch, "
         "TLV3702 + TL431, LVC1G32 + UCC27524, 12 V buck + 3.3 V LDO, 1 kΩ 3 W bleeder, 10 Ω RH-50 dummy load, 2 × PC817, 2-layer PCB or perfboard", 1, 40.0,
         "pre-charge / clamp interlock (one-shot on the contactor's opening edge), board tests before any motor", E),
        ("Bring-up IO (S1)", "sw/bringup.py `--estop gpiod:gpiochip0:17:27`: reset/ARM pushbutton, coil MOSFET + suppression, opto "
         "(aux contact → GPIO27), heartbeat lead GPIO17 → watchdog, 3 m 4-core cable for the remote mushroom, terminal blocks", 1, 20.0,
         "safety step (e-stop, watchdog, reset refused while pressed); operator outside the fall envelope", E),
        ("Commissioning (A5 v2)", "Category-0 catch: adjustable padded cradle (Al bar + EVA foam) + Dyneema tether ≥ 600 N + payload tray", 1, 25.0,
         "drop energy ≤ 3 J with ≤ 50 mm gap (electrical.md)", E),
        ("Bench", "Bench stand: 2 × 40×40 aluminium extrusion 400 mm + corner brackets + 2 bench clamps", 1, 30.0,
         "carries the Step-1 stage now, the shoulder later (Fable minor: no stand in the kit)", E),
        ("Step-1 rig (D13 v3)", "Steel rod Ø12 h6, 300 mm (stage shafts, cut 4 × 50 mm)", 1, 9.0, "the Step-1 yaw stage; the other cuts for pitch / roll / elbow", E),
        ("Step-1 rig (D13 v3)", "Aluminium HTD 5M pulley, 20T, 15 mm belt, **12 mm bore**, 2 set screws", 1, *AL_PULLEY[:1], "yaw small pulley", AL_PULLEY[1]),
        ("Step-1 rig (D24)", "Oldham coupling, D40, **12 mm clamp hub**, acetal (POM) disc", 1, *OLDHAM[:1], "yaw stage", OLDHAM[1]),
        ("Step-1 rig (D24)", "Acetal (POM) rod Ø40 × 100 mm", 1, 12.0, "spare Oldham discs, cut to fit the printed tongue", E),
        ("Step-1 rig", "HTD 5M-15 closed belt 315 mm (yaw, 20T:40T)", 1, *BELT[:1], "nearest stock to 313 (rnd/belts.py)", BELT[1]),
        ("Step-1 rig", "61808 (40×52×7) × 2 + 6801 (12×21×5) 10-pack", 1, 2 * B61808[0] + 10 * B6801[0], "yaw one-piece shaft (D19) + every D13 stage",
         "61808: AliExpress MOCHU 10-pack snapshot (loop 14); 6801 ESTIMATE"),
        ("Step-1 rig", "Test lever: 20 × 20 aluminium bar 300 mm + 1 kg and 2 kg weights + luggage scale", 1, 20.0,
         "torque constant, the drive's torque limit, the catch, the collision trip on one joint", E),
        ("Thermal (M7)", "10 × 10 k NTC thermistors (epoxy bead) + Kapton tape", 1, 6.0, "sleeve and rear-cap temperatures from day 1 (thermal.md)", E),
        ("Thermal (M7)", "Aluminium finned rear cap for the Step-1 GIM6010 cartridge", 1, *AL_CAP[:1],
         "measures the sleeve vs cap difference (thermal.md: closed sleeve 0.65 × rated continuous)", AL_CAP[1]),
        ("Fasteners (coupons)", "M4 brass heat-set inserts, **short** (Ø5.6 hole × 5.7 mm), 100-pack", 1, 15.93,
         "coupons first (insert pull-out, Fable C1/C3), then the arm", "Newark $15.93 / 100 (M4 × 8.1 Heat-Sert class; short variant ESTIMATE)"),
        ("Fasteners (coupons)", "M3 brass heat-set inserts (Ø4.0–4.1 hole), 100-pack", 1, 12.0, "coupons, then the arm", E),
        ("Fasteners (coupons)", "Stainless screw kit: M4 × 12 countersunk + button heads (Fable minor), M4 / M3 SHCS assortment, M5 × 30 + nylocs",
         1, 25.0, "Step-1 rig, coupons, later the cartridge flanges", E),
        ("Consumables", "Epoxy (CF bonds), grease, thread locker 243 / 270, PTFE washers", 1, 15.0, "", E),
        ("Filament (A6)", "PETG 3 kg + ASA 1 kg (cans, rear caps, Oldham tongue adapters) + TPU 95A 0.25 kg", 1, 90.0,
         "coupons + the Step-1 rig now, the arm in Step 2 (print_plan.md)", E),
    ]


def step2_common():
    """Step 2, independent of both holds: the rest of the arm."""
    return [
        ("Bearings", "61808 (40×52×7)", 4, *B61808[:1], "pitch and elbow one-piece shafts (D19), 2 each", B61808[1]),
        ("Bearings", "6708 (40×50×6) thin", 2, 2.5, "elbow belt cassette", E),
        ("Bearings", "61804 (20×32×7)", 2, 0.92, "D16 wrist-roll spindle", "AliExpress MOCHU snapshot; Zoro $4.69"),
        ("Bearings (M8)", "608-2Z bearings (C0 1.37 kN) with printed POM tyres on a Ø8 axle fork", 2, 2.5, "A9 balancer pulley A + tackle pulley (Fable M8: a U604 carried 634–748 N on C0 ≈ 0.3 kN; M8 track 3220161)", E),
        ("Joints (C1/M1 fixes)", "Pitch-hub tie-rod: 4 × M4 class 10.9 studs (DIN 976-1) + M4 nuts + 6061 clamp ring; yaw flange 6 × M3 × 45 12.9 + steel clamp ring",
         1, 12.0, "the arm hung from M3 inserts in pull-out (Fable C1; rnd/joints.py on the C1/M1 track)", E),
        ("Joints (C1/M1 fixes)", "Machined 6061-T6 yaw flange (hex spigot, M3 tapped at the hex corners)", 1, 25.0, "Fable C1 (C1/M1 track ESTIMATE)", E),
        ("Joints (C1/M1 fixes)", "7075-T6 balancer crank Ø44 × 4 with a pressed Ø6 pin, tapped for the tie-rod studs", 1, 20.0,
         "custom metal part missing from the kit (Fable minor)", E),
        ("Fasteners", "Hardened dowels Ø6 × 10 (yaw hard-stop pin) + Ø6 × 12 (3 × KX-V ferrule dowels)", 1, 6.0, "yaw stops ±155°, arm detach ferrule", E),
        ("Structure", "Carbon tubes Ø40×1 + Ø30×1 (one arm)", 1, 40.0, "upper-arm neck (bond ≈ 55 mm), forearm link (30 mm into each socket)", E),
        ("Structure", '3" quick-release squeeze tri-clamp, 304 SS', 1, 25.0, "KX-V ferrule at the shoulder (arm detach)", "hosewarehouse (Dixon 13MHHM-Q300)"),
        ("Balancer (A9)", "ISO 10243 green die spring Ø25 / Ø12.5 × L0 115, 18.7 N/mm (Berger Tools V25-115 or Accu TJL25-115) + 2 mm UHMWPE cord", 1, 14.0,
         "2:1 tackle balancer, pin B b 20 mm, a 40–60 mm (balancer.md)", E),
        ("Sensors (M3)", "4 × AS5048A joint encoders + 2 RP2040 CAN nodes (shoulder + elbow) + 2 tactile pads + palm ToF", 1, 96.0,
         "joint-side encoders; **no mount or node firmware yet** (Fable M3): buy with the arm, or drop the ≤ 1 mm accuracy claim for M1", E),
        ("Sensors (D14)", "Elbow node power + bus: 24→12 V **6 A** buck (two stalled STS3215 draw 5.4 A) + half-duplex TTL transceiver + 2 × MCP2562", 1, 14.0,
         "D14: 12 V and TTL made in the arm (sw/config/can_map.yaml)", E),
        ("Sensors", "Global-shutter wrist camera (OV9281 MIPI-CSI) + CSI→USB UVC bridge + slim high-flex USB cable", 1, 70.0, "D12 wrist camera", E),
        ("Harness", "Clock-spring ribbon stock: 20 AWG silicone pair, 26 AWG twisted pairs, Kapton/PET carrier tape; JST-GH/VH kit", 1, 18.0,
         "joint crossings (rnd/harness.py)", E),
        ("Filament (A6)", "PETG 2 kg more (the arm itself after the coupons)", 1, 40.0, "one arm ≈ 3 kg (print_plan.md)", E),
    ]


# Hold A (D20): yes = the seller confirms 24 V operation (speed, torque) of the GIM8108-8's driver version.
HOLD_A = dict(
    title="Hold A: GIM8108-8 at shoulder pitch + elbow (D20)",
    waits="the seller's confirmation that the GIM8108-8 driver version runs at 24 V (rated torque and speed at 24 V), plus its drawing; "
          "and the D20 drive redesign (Fable M6)",
    yes=[("Actuators", "SteadyWin GIM8108-8, **24 V-capable driver version** (confirm which: SDC104 / GDZ468 / GDS68)", 2, GIM8108_8[0],
          "shoulder pitch (3:1) + elbow (3:1, 20T:60T): 1.8× rated on the 20 lb hug (R2c)", GIM8108_8[1]),
         ("Drive (D20, M6)", "Oldham coupling **Ø50**, 15 mm clamp hub, rated ≥ 25 N·m", 2, 22.0, "pitch + elbow (cartridge_m D20: Ø50 at pitch and elbow)", E_D20),
         ("Drive (D20, M6)", "Steel rod Ø15 h6, 300 mm + 2 × 5 mm parallel keys (keyway machined)", 1, 18.0,
          "a Ø15 keyed stage: 22 N·m at the 20T pulley is 3.7 kN at a Ø12 clamp (Fable M6)", E_D20),
         ("Drive (D20, M6)", "Aluminium HTD 5M pulley, 20T, **25 mm** belt, Ø15 keyed bore", 2, 14.0, "pitch + elbow small pulleys", E_D20),
         ("Drive (D20, M6)", "61902 (15×28×7) bearings, C0 ≈ 2.3 kN", 4, 2.5, "2 per stage: 0.7 kN each at the 1.38 kN peak pull (a 6801 has C0 ≈ 1.0 kN)", E_D20),
         ("Drive (D20, M6)", "HTD 5M **25 mm** closed belts: pitch (20T:60T) + elbow (20T:60T, length from the D20 CAD), or steel-cord", 2, 14.0,
          "1382 N peak pull at 22 N·m is 2.8× a 15 mm belt's ultimate/3 (belts.py D20)", E_D20),
         ("Thermal (M7)", "Aluminium finned rear caps, Ø96 class", 2, 18.0, "pitch + elbow GIM8108-8 cartridges", E)],
    no=[("Actuators", "SteadyWin GIM6010-8 24 V SDC104", 2, GIM6010[0], "pitch + elbow as today (bench-valid at 1 kg; 0.6–0.8× on the 20 lb hug)", GIM6010[1]),
        ("Drive (D24)", "Oldham coupling, D40, 12 mm clamp hub, POM disc", 2, OLDHAM[0], "pitch + elbow", OLDHAM[1]),
        ("Drive (D13 v3)", "Aluminium HTD 5M pulley, 20T, 15 mm belt, 12 mm bore", 2, AL_PULLEY[0], "pitch + elbow small pulleys", AL_PULLEY[1]),
        ("Drive", "HTD 5M-15 closed belts 355 (pitch, 20T:60T) + 295 (elbow, 20T:40T) mm", 2, BELT[0], "pitch + elbow", BELT[1]),
        ("Thermal (M7)", "Aluminium finned rear caps, Ø86 class", 2, AL_CAP[0], "pitch + elbow GIM6010 cartridges", AL_CAP[1])],
    note="Fable M6: the D20 swap is a drive redesign, not a motor swap. With 22 N·m at the 20T pulley the repo's own check gives 1382 N "
         "peak belt pull (2.8× a 15 mm belt's ultimate/3), ≈ 0.7 kN on each 6801 (C0 ≈ 1.0 kN) and 3.7 kN at a Ø12 clamp and set screws; "
         "the cartridge needs Ø50 Oldhams (cartridge_m.py). The D20 lines are ESTIMATE until the cartridge track's D20 parts list lands. "
         "The 24 V question is the other gate: most GIM8108-8 listings are 48 V nominal.",
    hi=lambda: 2 * (GIM8108_8_HI - GIM8108_8[0]) + line_sum(HOLD_A["yes"]) - line_sum(HOLD_A["no"]),
)

# Hold B (D24): the inline roll is the plan (owner OK 2026-11-27); the belt roll is the fallback if the drawing or the
# protocol fails.
HOLD_B = dict(
    title="Hold B: inline GIM8108-36 at shoulder roll (D24, **approved by the owner 2026-11-27**)",
    waits="the seller's answers (output face, output-bearing radial rating ≥ 280 N, backlash, SDC303 protocol and CAN timeout) and "
          "the Step-1 protocol result; the *no* list is the fallback (belt roll)",
    yes=[("Actuators", "SteadyWin GIM8108-36, **SDC303 (24 V)** driver", 1, GIM8108_36[0], "shoulder roll, inline 36:1", GIM8108_36[1]),
         ("Bearings", "61806 (30×42×7)", 1, 3.0, "rear roll support on the roll link's stub (68 mm from the motor's output bearing)", E),
         ("Drive (D24 inline)", "Output adapter, 7075-T6 Ø76 × 7 (CNC: Ø30 pilot, 3 × Ø4 H7 reamed, 3 × M4 csk, 8 × M5 tapped on PCD 64)", 1, 30.0,
          "motor Ø30 boss → printed clevis arm; torque on the 3 dowels (1.6× stall, rnd/joints.py on the C1/M1 track)",
          "ESTIMATE: JLCCNC / PCBWay-class instant quote ≈ $20–25 each at qty 2 + shipping share"),
         ("Drive (D24 inline)", "Ø80/Ø52 × 2 steel clamp ring + 8 × M5×16 DIN 7984 10.9, 3 × M4×10 ISO 10642, 3 × Ø4 m6 dowels", 1, 9.0,
          "adapter ↔ clevis arm clamp, boss ↔ adapter", E),
         ("Drive (D24 inline, M1)", "Rear mount: 8 × M4 studs (Loctite 270) + nuts instead of M4 × 18 into 3.5 mm threads; 2 × hardened Ø10 × 28 stop pins + stainless wear strips",
          1, 8.0, "Fable M1: printed Ø8 stops saw 16 MPa at the limit, 2.3 kN at stall; no stripped $240 motor", E)],
    no=[("Actuators", "SteadyWin GIM6010-8 24 V SDC104", 1, GIM6010[0], "shoulder roll on the 18T:80T belt (4.44:1)", GIM6010[1]),
        ("Drive (D13 v3)", "Aluminium HTD 5M pulley, 18T, 15 mm, 12 mm bore", 1, AL_PULLEY[0], "roll small pulley", AL_PULLEY[1]),
        ("Drive (D24)", "Oldham coupling D40, 12 mm clamp hub, POM disc", 1, OLDHAM[0], "roll stage", OLDHAM[1]),
        ("Drive", "HTD 5M-15 closed belt 475 mm", 1, BELT[0], "roll (80T)", BELT[1]),
        ("Bearings", "61808 (40×52×7)", 2, B61808[0], "roll one-piece shaft (D19)", B61808[1]),
        ("Thermal (M7)", "Aluminium finned rear cap, Ø86 class", 1, AL_CAP[0], "roll GIM6010 cartridge", AL_CAP[1])],
    note="The CAD and checks for both drives are in the tree (`PIP_ROLL_DRIVE=inline`, roll_inline.md). Inline-specific risks (Fable M1): "
         "the output boss joint is carried by its dowels (1.6× stall), the stops must be steel, and the 36:1 planetary adds ≈ 15 arcmin of backlash "
         "(accuracy.md: 4.3 mm at the hand without joint encoders). The SDC303 is a different driver from the Step-1 SDC104: protocol discovery "
         "runs again on it on arrival.",
    hi=lambda: line_sum(HOLD_B["yes"]) - GIM8108_36[0] + GIM8108_36_HI - line_sum(HOLD_B["no"]),
)

# Exact questions for the seller (SteadyWin reseller: aifitlab / Robotics Center / openelab), one message.
SELLER_QUESTIONS = [
    "**GIM8108-8, 24 V:** we run a 24 V bus (6S Li-ion, 25.2 V full; our regen clamp holds it ≤ 26.5 V, absolute max 28 V). Which driver "
    "version of the GIM8108-8 (SDC104, GDZ468, GDS68?) do you ship, and is it specified for 24 V? Please give rated torque, rated speed, "
    "no-load speed and continuous current at 24 V (listings quote 48 V). Price and lead time for 2 now (4 later).",
    "**GIM8108-8 drawing:** installation drawing with frame diameter and height, output-flange bolt pattern (hole count, PCD, thread, depth) "
    "and housing / rear mounting pattern, and the connector position.",
    "**GIM8108-36 (SDC303, 24 V):** which face is the output; the output flange material and the dowel-hole depth; reducer **backlash** "
    "(listings say 15 arcmin: measured how?); frame and connector position (we have the Ø96 × 57 drawing).",
    "**GIM8108-36 output bearing:** bearing type and allowable radial load at the output flange: we need **≈ 280 N radial** continuous, "
    "with a second support bearing 68 mm behind the flange; also the allowable axial load and tilting moment (one listing gives 1000 N "
    "radial / 250 N axial: please confirm and say where it is measured). Price with the SDC303 driver and lead time.",
    "**CAN protocol manual** for the SDC104 (GIM6010-8), SDC303 (GIM8108-36) and the GIM8108-8's driver: frame formats; is the MIT / "
    "impedance mode (position, velocity, kp, kd, torque feed-forward) the default or selected in the tool, and its float ranges; how to "
    "set the node ID and the feedback ID; 1 Mbit/s; a **CAN-command timeout** (the drive disables itself when commands stop) and its "
    "unit; a torque / current-limit parameter **saved to flash**; encoder zero (is the output angle kept over a power cycle?); status and "
    "error codes; under/over-voltage behaviour. **Is it the same protocol on all three drivers?**",
    "**GIM6010-8:** confirm the unit is the 24 V **SDC104** version (not a 48 V listing), with its installation drawing and which face "
    "carries the housing mounting holes (rear or output face: our cartridge cap depends on it, Fable C3).",
]


def sd(x):
    """Signed dollars: +$57 / −$12."""
    return f"{'+' if x >= 0 else '−'}${abs(x):,.0f}"


def line_sum(L):
    return sum(q * u for _, _, q, u, _, _ in L)


def items():
    """Step 1 (the list that can be bought now); kept for callers of the old `items()`."""
    return step1()


CHANGES = [
    "**Two steps (Fable verdict):** Step 1 is one GIM6010-8, the DM-J4310, the two STS3215, the CAN adapters, the bench supply, the whole "
    "safety board, one D13 stage on a test lever, thermistors + one aluminium cap, inserts / screws for the coupons and filament. Everything "
    "else waits for Step 1's exit criteria. The \"zero-regret 3 extra GIM6010s\" advice is gone.",
    "**Bench supply (M2):** Mean Well SE-1000-24 (41.7 A) for the arm's 14.8 A sustained / 31.5 A peak; the 10 A CC supply stays for the 1 A "
    "first power-up; fuses 40 A main, 20 A arm lead.",
    "**Safety board (M2):** netlist + block diagram in electrical.md; the forced discharge is a one-shot on the contactor's opening edge "
    "(it fought the pre-charge); 3 clamp resistors; logic parts, bleeder and a dummy load are new lines.",
    "**Hold A (M6):** Ø50 Oldhams, a Ø15 keyed stage, 25 mm belts, 61902 bearings and Ø96 caps are in the *yes* list (ESTIMATE until the "
    "cartridge track's D20 list), so the D20 delta is no longer motors only.",
    "**Inline roll (M1):** studs + nuts on the rear mount and hardened stop pins in the *yes* list; C1 tie-rod hardware, the 6061 yaw flange "
    "and the 7075 balancer crank in Step 2; 608-class balancer pulleys (M8).",
    "**Thermal (M7):** NTC thermistors from day 1 and an aluminium finned rear cap per GIM cartridge (thermal.md: a closed ASA sleeve allows "
    "0.65 × rated continuous).",
]

# On arrival / before paying for Step 1
CHECKS = [
    "GIM6010-8: the **24 V SDC104** version; DM-J4310-2EC: **V1.1** (ask for the output bearing's radial / moment rating: the wrist "
    "pitch sees ≈ 0.9 N·m bending, more with the basket hook); STS3215: the **12 V C018**.",
    "SE-1000-24: the 24 V model (not -12 / -48); check the overload mode on its datasheet (constant-current limiting) and set 24.0–25.2 V.",
    "Aluminium HTD 5M pulleys: 15 mm belt, **12 mm bore** (many default to 8 mm), two set screws.",
    "Oldham couplings: **12 mm clamp-style** hub, OD ≤ Ø42 (the cartridge nose bore is Ø43); measure the disc (thickness, slot width and "
    "depth) and set `OLDHAM` in rnd/parts/cartridge_m.py before printing the tongue adapters.",
    "Heat-set inserts: the short M4 (5.7 mm) the flange calls for; the long 8.1 mm ones don't fit the 6 mm flange.",
]


def totals():
    c = 1 + CONTINGENCY
    s1, s2 = line_sum(step1()), line_sum(step2_common())
    a_yes, a_no = line_sum(HOLD_A["yes"]), line_sum(HOLD_A["no"])
    b_yes, b_no = line_sum(HOLD_B["yes"]), line_sum(HOLD_B["no"])
    plan2 = s2 + a_no + b_yes                     # today's plan: GIM6010 at pitch + elbow, inline roll
    return dict(s1=s1, s1_c=s1 * c, s2=s2, a_yes=a_yes, a_no=a_no, b_yes=b_yes, b_no=b_no,
                s2_plan=plan2, s2_plan_c=plan2 * c, s2_d20_c=(s2 + a_yes + b_yes) * c, s2_belt_c=(s2 + a_no + b_no) * c,
                d_a=(a_yes - a_no) * c, d_b=(b_yes - b_no) * c, d_a_hi=HOLD_A["hi"]() * c, d_b_hi=HOLD_B["hi"]() * c,
                plan_c=(s1 + plan2) * c, d20_c=(s1 + s2 + a_yes + b_yes) * c, belt_c=(s1 + s2 + a_no + b_no) * c,
                # old keys (m1_readiness / bom callers)
                now=s1, now_c=s1 * c, default=(s1 + s2 + a_no + b_no) * c, both=(s1 + s2 + a_yes + b_yes) * c)


def _table(L, rows):
    L += ["| Group | Item | Qty | Unit $ | Line $ | Why | Price source |", "|---|---|---|---|---|---|---|"]
    L += [f"| {g} | {n} | {q} | {u:,.2f} | {q * u:,.0f} | {w} | {s} |" for g, n, q, u, w, s in rows]


def _sub(L, x, label):
    c = CONTINGENCY
    L += [f"| | **{label} subtotal** | | | **{x:,.0f}** | | |", f"| | Contingency {c:.0%} (A16) | | | {x * c:,.0f} | | |",
          f"| | **{label} total** | | | **{x * (1 + c):,.0f}** | | |", ""]


def main():
    S1, S2 = step1(), step2_common()
    t = totals()
    c = CONTINGENCY
    est = lambda rows: sum(1 for *_, s in rows if s.startswith("ESTIMATE"))
    L = ["# Milestone M1: one-arm bench kit in two steps (generated by `python -m rnd.m1_kit`)", "",
         f"**Step 1 (one joint: GIM protocol + safety chain): ${t['s1']:,.0f} + {c:.0%} = ${t['s1_c']:,.0f}.** "
         f"**Step 2 (the rest of the arm, today's plan): ${t['s2_plan']:,.0f} + {c:.0%} = ${t['s2_plan_c']:,.0f}.** "
         f"Whole kit ${t['plan_c']:,.0f} with contingency (the previous list: $1,945, with a 10 A supply, no safety-board logic, a motors-only hold A and none of the C1/M1/M7 fix parts).", "",
         "| Scenario (with contingency) | Step 2 | Whole kit |", "|---|---|---|",
         f"| **Today's plan**: GIM6010 at pitch + elbow, approved inline roll | ${t['s2_plan_c']:,.0f} | **${t['plan_c']:,.0f}** |",
         f"| Hold A yes (D20, with its drive parts) + inline roll | ${t['s2_d20_c']:,.0f} | ${t['d20_c']:,.0f} |",
         f"| Fallback: belt roll (hold B no), GIM6010 at pitch + elbow | ${t['s2_belt_c']:,.0f} | ${t['belt_c']:,.0f} |", "",
         f"Hold A (D20) costs **{sd(t['d_a'])}** against the GIM6010 lines (up to {sd(t['d_a_hi'])} at the dearest motor listing); the inline roll "
         f"{sd(t['d_b'])} against the belt roll (up to {sd(t['d_b_hi'])}). Step 1: {len(S1) - est(S1)} of {len(S1)} lines priced from retailer snapshots, "
         f"{est(S1)} ESTIMATE. Everything except the two supplies, the stand, the test lever and the catch is reused in the arm and the robot.", "",
         "**Why two steps (Fable review 2026-11-27, verdict):** no GIM can be commanded until its CAN protocol is known, the arm mixes three "
         "SteadyWin drivers (docs/sw/gim_protocol_notes.md), and the arm CAD has fixes to land before printing (C1 insert pull-out, C3 rear cap, "
         "M1 inline boss/stops, M5 belt centres). Step 1 settles the protocol and the safety chain on one real GIM with only one GIM bought; the bench supply and the safety board are most of its cost and any arm needs them; "
         "Step 2 is bought when its exit criteria are met ([M1_READINESS.md](M1_READINESS.md)). **No extra GIM6010s in Step 1.**", "",
         "**What Step 1 proves on hardware:** the SDC104's CAN protocol (`python -m sw.gim_discover`: MIT / impedance mode or not), enable, zero-gain "
         "and impedance control, the drive's torque limit in flash and its CAN timeout; the safety board (pre-charge, one-shot discharge, clamp, "
         "contactor, watchdog at 25 ms, e-stop → no torque ≤ 50 ms) with a real drive; torque constant and friction on a lever; the collision "
         "trip on one joint; the sleeve vs aluminium-cap temperature; insert pull-out and the bolted-flange coupons.", "",
         "**What Step 2 proves:** the whole arm: belts and the D13 v3 stages, the bolted cartridges, the balancer (A9), the inline roll, hand "
         "sag (R2b) and, if the encoder mounts and node firmware exist by then (Fable M3), hand accuracy; the fin-ray grasp; CAN timing (SW1).", "",
         "## Step 1: buy now (one joint, the protocol and the safety chain)", ""]
    _table(L, S1)
    _sub(L, t["s1"], "Step 1")
    L += ["## Step 2: the rest of the arm (after Step 1's exit criteria)", "", "### Common to every Step-2 scenario", ""]
    _table(L, S2)
    L += [f"| | **Common subtotal** | | | **{t['s2']:,.0f}** | | |", ""]
    for k, H in (("a", HOLD_A), ("b", HOLD_B)):
        y, n = t[f"{k}_yes"], t[f"{k}_no"]
        L += [f"### {H['title']}", "", f"**Waits on:** {H['waits']}.", "", f"**If yes, buy** (${y:,.0f}):", ""]
        _table(L, H["yes"])
        L += ["", f"**If no, buy instead** (${n:,.0f}):", ""]
        _table(L, H["no"])
        L += ["", f"**Delta: {sd(y - n)} before contingency, {sd(t[f'd_{k}'])} with it** (at the dearest motor listing {sd(t[f'd_{k}_hi'])}). {H['note']}", ""]
    L += [f"**Step 2, today's plan (common + hold A *no* + hold B *yes*): ${t['s2_plan']:,.0f} + {c:.0%} = ${t['s2_plan_c']:,.0f}.**", "",
          "## Questions for the seller (send as one message now; they gate Step 2's motors)", ""]
    L += [f"{i}. {q}" for i, q in enumerate(SELLER_QUESTIONS, 1)]
    L += ["", "Ask Damiao (or the DM-J4310 reseller) separately: the V1.1 output bearing's allowable radial load and moment.", "",
          "## What changed since the loop-47 list", ""] + [f"- {x}" for x in CHANGES] + [
          "", "## Check before paying / on arrival", ""] + [f"- {x}" for x in CHECKS] + [
          "", "**Order of work:** build and test the safety board (electrical.md, bench tests before any motor) → protocol discovery on the "
          "GIM6010-8 → bring-up on the Step-1 rig (`--joints yaw,wpitch`) → coupons (insert pull-out, bolted flange + Oldham tongue: "
          "[COUPONS.md](COUPONS.md)) → Step 2. Readiness and the go / no-go per step: [M1_READINESS.md](M1_READINESS.md). Buying is the owner's decision."]
    (DOCS / "M1_BENCH_ARM.md").write_text("\n".join(L) + "\n")
    print("\n".join(L[2:10]))
    return t


if __name__ == "__main__":
    main()
