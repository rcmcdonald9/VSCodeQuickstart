"""Milestone M1: one-arm bench kit, bought in two steps (Fable review 2026-11-27: buy with changes).
python -m rnd.m1_kit

- **Step 1: prove the GIM protocol and the safety chain on ONE joint.** One GIM6010-8 (the yaw motor later), the DM-J4310,
  the CAN adapters, the bench supply and the whole safety board, a test lever bolted straight to the GIM output, the
  joint's turned C45 drive stub (D26: no Oldham) for the stub coupon, thermistors, inserts and screws for the coupons,
  filament. Nothing in it waits on a
  seller answer or a CAD fix, and every part is reused in the arm (except the bench tools). Neon review #2 finding 15
  (2026-11-27): the D40 Oldham / plain Ø12 / set-screw pulley / 315 belt stage (belts.md: 0.86× / 0.65× at the motor peak,
  yaw centre 93 % of its adjuster), the unverified CNC rear cap (seller question 6) and the two STS3215 (no 12 V rail
  before the elbow node's buck) left Step 1; the stage is bought in Step 2 (D26: a direct stub stage, no Oldham).
- **Step 2: the rest of the arm**, bought only when Step 1's exit criteria are met (docs/M1_READINESS.md): the common arm
  parts, plus two holds that each list what they buy if the answer is yes and what they buy instead if no:
  - **Hold A (D20)**: GIM8108-8 at shoulder pitch + elbow, waiting on the seller's 24 V confirmation; **not motors only**
    (Fable M6): the drive parts for 22 N·m at the 20T pulley (D26: steel stubs with 10.9 screws, steel 20T, steel-cord belts).
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
# W10 BOM consolidation (2026-11-27): the W8b cartridge parts (cartridge_m.parts_list), W8c load path, W8d tip parts.
# Oldham D50 per GIM6010 stage = the shaft-side keyed clamp hub + the acetal disc (sold separately) + the metal motor-side
# hub (the printed ASA tongue was rejected in W8b: 0.74× friction at the peak). The generic $20 set of the earlier lists
# had no source, no keyway and no torque rating; AliExpress could not be reached from this session (host not allowed).
OLDHAM = (67.19 + 36.45, "Zoro: Ruland MOCC51-12-A clamp hub (12 mm bore, 4 mm keyway, Ø50.8, aluminium) $67.19 + OD32/51-AT acetal disc $36.45 "
                         "(2026-11-27 search snapshots); disc torque rating to read off Ruland's datasheet (the RS '9.5 N·m' is the clamp-screw seating torque)")
OLDHAM_MOTOR_HUB = (15.0, "ESTIMATE: 6061-T6 from the drive_adapter STEP (Ø50 × 4 flange + tenon, 4 × Ø3.4 + c'bore on PCD 34), JLCCNC / PCBWay-class "
                          "instant quote at qty 8; reworking a 2nd bought hub is a $54.19 MOCT51-12-A (Zoro) + lathe time")
# Cheapest credible alternatives (owner decision on the $5,500 limit; rnd/bom.py drivers()):
OLDHAM_ALT = (25.0 + 12.0, "Huco 452H50.35 Ø50 hub, 12 mm bore with keyway, £19.43–20.21 ex VAT at RS UK (€23.37 RS IE, A$45.79 RS AU; confirm the clamp "
                           "variant and its torque rating) + Huco acetal disc ≈ $12 ESTIMATE (sold separately, not listed); the motor-side hub stays the 6061 CNC part")
D63_SET = (130.0, "ESTIMATE: D63 shaft-side M5 clamp hub 12 H7 + 4 mm keyway + acetal disc, ≥ 25 N·m; scaled from the Ruland Ø50.8 hub + disc $103.64 "
                  "(Zoro); Ondrives 2-1/4\" clamp Oldham $84–157 (2026-11-27 search)")
D63_MOTOR_HUB = (18.0, "ESTIMATE: 6061-T6 D63 × 4 flange + tenon, 4 × Ø3.4 + c'bore on PCD 30 (CNC, as the D50 hub)")
M3X6_PACK = (10.0, "ESTIMATE: DIN 7984 M3 × 6 8.8 zinc, 100-pack (D26: 8 per drive stub)")
M3X6_109 = (8.0, "ESTIMATE: DIN 7984 M3 × 6 10.9, 50-pack (D20 stubs: 8 each at 22 N·m)")
# D26 (owner: cost-down; the Oldham couplings go): the pulley sits on a turned C45 stub bolted to the motor output (8 × M3 × 6),
# the stub's far end in one bearing in wall B (belt_stage.small_stage_v4); the pitch belt is tensioned by a slotted idler.
STUB = (18.0, "ESTIMATE: turned + milled C45 / 1045 drive stub (Ø42 × 4 flange, 8 × Ø3.4 + c'bore, Ø20 pilot, Ø12 h6 × 63 shaft, 4 × 2.5 keyway, "
              "k6 bearing seat), qty 7–10; basis: JLCCNC lists CNC turning / milling from $5.00 a part (jlccnc.com, 2026-11-27 search) + a milling "
              "op for the keyway and the 8-hole pattern + C45 bar; no published per-part price without a CAD upload (Xometry / Hubs / RapidDirect "
              "quote from the STEP only), so ESTIMATE until the STEP is quoted")
B6001 = (3.45, "eBay VXB 10 × 6001 (12×28×8) $34.49 pro rata (2026-11-27 search snapshot; RS PRO 6001-2RS 10-pack £19.53 ex VAT, Zoro Alto $6.89 each)")
B61901 = (3.85, "Zoro Tritan 61901-2RS $3.85 each (2026-11-27 search snapshot; ZEN £2.82 ex VAT at Bearing Boys)")
B625 = (0.99, "OpenBuilds 625-2RS $0.99 each (2026-11-27 search snapshot; RS PRO 10-pack £17.20 ex VAT)")
IDLER_HW = (1.5, "ESTIMATE: ISO 4762 M5 × 40 (cut to 38) + captive M5 nut + 2 Ø8 × 5.3 spacers + M4 × 20 jack screw + nut")
STRAP = (4.0, "ESTIMATE: 3 mm 6061-T6 strap 70 × 14 + 4 holes, rides on the SendCutSend-class chassis-ring order")
STRAP_HW = (1.0, "ESTIMATE: 4 × M4 × 12 ISO 4762 + 2 M4 heat-set inserts (from the 100-pack)")
SLEEVE = (7.0, "ESTIMATE: turned 42CrMo4+QT, Ø ± 0.02, length = print − 0.05; JLCCNC lists CNC turning from $5.00 a part (jlccnc.com, "
               "2026-11-27 search), + ≈ $2 for the QT bar and the length tolerance; US instant-quote shops (Xometry / Hubs class) cost several times this")
STOP_BUSH_P = (10.0, "ESTIMATE: stepped Ø10 / Ø7 bush, Ø7.2 / Ø4.3 bores, 42CrMo4+QT, turned (basis as the sleeves)")
PLATE_RING = (10.0, "ESTIMATE: 3 mm 7075-T6 ring Ø96 / 70 + 8 holes, SendCutSend 7075-T6 .125\" (their .25\" aluminium 4 × 2\" example $13.33 at 1, $5.30 at 10)")
LOCTITE = (28.99 + 20.44, "Loctite 660 50 ml $28.99 (Walmart, 'Press Fit Repair 660'; Ellsworth $59.82) + Loctite 638 10 ml $20.44 (Hisco / TestEquity), 2026-11-27")
ELBOW_TUBE = (15.0, "ESTIMATE: 6061-T6 Ø30 × 1.5 metric tube, 0.5 m (Zoro 6061 1.5\" × 0.083\" is $100.99 / 6 ft ≈ $17 / ft, the imperial reference)")
EPOXY = (25.0, "ESTIMATE: structural epoxy, Loctite EA 9466 / 3M DP420 class 50 ml cartridge + nozzles")
BALL2 = (45.0, "ESTIMATE: 2\" (Ø50.8) steel-ball transfer unit; 2\" units are heavy-duty industrial lines (Motion: General Bearing 41200 series, quote "
               "required); 1-1/2\" Hudson BT-1-1/2 CS is $29.75 at Penn Tool, $20.67 on eBay (2026-11-27 search)")
# D26 cost-down (wave 11): stock tube for the low-stress crush sleeves, PTFE skid feet for the D18b casters, saw-cut ballast.
SLEEVE_TUBE = (10.0, "ESTIMATE: one 500 mm stick of seamless E235+C precision tube (EN 10305-1) per size, 8 × 1.2 and 6 × 0.8; eBay.de-class "
                     "listings from ≈ €1.5 a stick (E235+C Ø3–20, 500 mm, 2026-11-27 search snapshot; host not reachable from this session) + a share "
                     "of shipping to the US")
FACING_JIG = (15.0, "ESTIMATE: mini tube cutter + 400 / 600-grit on a flat plate + a drilled steel guide block per sleeve OD (the 0–25 mm micrometer is a tool)")
SLEEVE_ALT = (0.6, "ESTIMATE: 1/4\" × .035\" 4130 tube, normalized (F_ty ≥ 75 ksi), Aircraft Spruce 03-00300 by the foot (price not shown in the "
                   "2026-11-27 snapshot; Zoro TW Metals 5 ft $91.49), ≈ 32 mm a sleeve")
PTFE_GLIDES = (17.99, "Target: Unique Bargains PTFE screw-on furniture glides 0.75\" (19 mm), 20 pcs, regular $17.99 (sale $13.49 on 2026-11-27, "
                      "Black Friday, end date not shown)")
PTFE_TAPE = (10.0, "ESTIMATE: skived PTFE adhesive tape 0.1 mm × 25 mm, 5 m roll (flank of the 4 skid feet)")
BALLAST_BAR = (25.0, "ESTIMATE: two 92 mm saw-cut drops of 1-3/4 × 3\" A36 hot-rolled flat bar (10.8 lb) from a local steel yard at ≈ $1.5 / lb + "
                     "a cut charge; online: Speedy Metals-class ≈ $2.9 / in for 1-1/4 × 1-3/4 (2026-11-27 snapshot, ≈ $50 for ours + shipping), "
                     "Zoro Online Metal Supply 1-3/4 × 3 × 9\" $102.99")
SOLENOID = (12.99, "Walmart: JF-1039 24 V 400 mA, 10 mm stroke, 25 N push-pull solenoid $12.99; Fielect JF-1039-24V $67.99 elsewhere (2026-11-27 search)")
BALLAST_PLATE = (7.0, "SendCutSend mild steel: their .25\" HRP&O 4.9 × 2\" example is $19.89 at 1, $7.02 at 10, $4.89 at 100 (2026-11-27 search); ours 4.0 × 2.8\", 14 off")
KEYED_SHAFT = (15.0, "ESTIMATE: Ø12 h6 C45 ground rod 300 mm with a 4 × 2.5 DIN 6885 keyway (cartridge_m.parts_list)")
KEYS = (5.0, "ESTIMATE: DIN 6885 A 4 × 4 key assortment (D26: one 16 mm key per stage, under the pulley)")
KEYED_PULLEY = (12.0, "ESTIMATE: aluminium HTD 5M 20T, 15 mm belt, 12 H7 bore + 4 mm keyway, set screw on the key (belts.md 1.41×)")
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
        ("Electronics (A14)", "USB-CAN adapter, candleLight / gs_usb (FYSETC UCAN, STM32F072), onboard 120 Ω (jumper)", 2, 19.0,
         "can_map.yaml buses B1 + B2 = SocketCAN can0 / can1", "MatterHackers $19 (2026-11)"),
        ("Electronics (A14)", "Waveshare 2-CH CAN FD HAT (2 × MCP2518FD, isolated, SKU 17075): SPI-CAN alternative, measured "
         "against the two gs_usb adapters on the Step-1 bus (docs/sw/can_interface_note.md)", 1, 46.99,
         "decides the robot's 5 CAN channels (A14): latency, jitter, loss, bus-off on the Pi 5; check its INT GPIOs vs 17/27",
         "waveshare.com $46.99 (2026-11-27)"),
        ("Electronics", "Raspberry Pi 5 Active Cooler (official; heatsink + PWM fan)", 1, 5.0, "no thermal throttling under the 500 Hz loop (Menlo sim2real lesson: throttling quietly degraded actuator response)", "raspberrypi.com list price $5 (ESTIMATE until a retailer snapshot)"),
        ("Power (M2)", "**Mean Well SE-1000-24** (24 V 41.7 A, 1 kW), set 24.0–25.2 V", 1, *PSU[:1],
         "arm 14.8 A sustained / 31.5 A peak at the rated drive limit; overload is a latched shutdown at 43.8–52 A, not current limiting (electrical.md *Bench supply*)", PSU[1]),
        ("Power (A5)", "Adjustable 0–30 V / 10 A CC/CV bench supply", 1, 65.71, "the 1 A current-limited first power-up only (drives disabled)",
         "Newegg generic 30 V 10 A CC/CV $65.71 (2026-11-27)"),
        ("Power (M2)", "4,700 µF 35 V bus capacitor + **60 A MIDI** main fuse (F1, sized for the robot pack) and 20 A ATO arm-lead fuse with holders + 10 / 14 AWG silicone wire, XT60",
         1, 30.0, "electrical.md safety-board netlist F1, F3, C1 (Neon #2 F1: a 40 A ATO never clears on the 41.7 A supply)", E),
        ("Power (A5 v2)", "Regen clamp / bus discharge: 3 × **Vishay NH-50 0.56 Ω (non-inductive)** + 3 × **VS-40CPQ060-N3 freewheel Schottky** + 3 × IRFB7530 + gate driver + gate resistors, film caps, 85 °C thermal switch, heatsink plate",
         1, 127.0, "mandatory on a bench supply; Neon #2 F1: the wirewound RH-50 with no freewheel path avalanched the MOSFETs (electrical.md)",
         "ESTIMATE: NH-50 $30 each (IBS NH05050R00FE02 $15.01, Future NH05025R00FE02 $42.30); VS-40CPQ060-N3 RS £37 / 25; rest as before"),
        ("Power (A5 v2)", "E-stop: 2 two-channel latching mushrooms + TE ALEV100 DC contactor (24 V coil, aux contact) + K_PRE relay + 3.3 Ω RH-50 pre-charge "
         "+ SN74LVC1G123 watchdog on its own LDO", 1, 150.0, "category-0 stop; watchdog 25 ms (sized on sw/loop_jitter.py)",
         "LEV100A5ANG $92 (Master Electronics); rest ESTIMATE"),
        ("Power (M2)", "Safety-board logic (netlist v3 in electrical.md): LM74700-Q1 + IRFB7530 ideal diode, 2nd LVC1G123 (discharge one-shot on Ā), LVC1G74 latch + power-on reset, "
         "**TLV3202** + TL431B, LVC1G11 + LVC1G32 (K1 hold on AUX), LVC1G04, UCC27524, 12 V buck + 3.3 V LDO, 1 kΩ 3 W bleeder, 10 Ω RH-50 dummy load, 2 × PC817, 2-layer PCB", 1, 45.0,
         "**order after the KiCad schematic + LTspice clamp simulation are reviewed** (M1_READINESS Step 1 gate); board tests before any motor", E),
        ("Bring-up IO (S1)", "sw/bringup.py `--estop gpiod:gpiochip0:17:27`: reset/ARM pushbutton, coil MOSFET + suppression, opto "
         "(aux contact → GPIO27), heartbeat lead GPIO17 → watchdog, 3 m 4-core cable for the remote mushroom, terminal blocks", 1, 20.0,
         "safety step (e-stop, watchdog, reset refused while pressed); operator outside the fall envelope", E),
        ("Commissioning (A5 v2)", "Category-0 catch: adjustable padded cradle (Al bar + EVA foam) + Dyneema tether ≥ 600 N + payload tray", 1, 25.0,
         "drop energy ≤ 3 J with ≤ 50 mm gap (electrical.md)", E),
        ("Bench", "Bench stand: 2 × 40×40 aluminium extrusion 400 mm + corner brackets + 2 bench clamps", 1, 30.0,
         "carries the Step-1 lever now, the shoulder later (Fable minor: no stand in the kit)", E),
        ("Step-1 rig", "Test lever: 6061 flat bar 40 × 6 × 300 mm, drilled on arrival to the GIM6010-8's measured output pattern and bolted straight to it "
         "+ 1 kg and 2 kg weights + luggage scale", 1, 25.0,
         "protocol, torque constant, friction, the drive's torque limit, the catch, the collision trip on one joint: no belt stage needed (Neon #2 F15)", E),
        ("Drive (D26)", "Turned **C45 drive stub** for the Step-1 GIM6010 joint (belt_stage.STUB: Ø42 × 4 flange, 8 × Ø3.4 + c'bore on the measured output PCD, "
         "Ø12 h6 shaft + 4 mm keyway, k6 seat for the wall-B 6001)", 1, *STUB[:1],
         "the yaw stage's stub, reused in Step 2; now the stub coupon: 8 × M3 × 6 + Loctite 243 on the GIM output to 1.5 × the 11 N·m peak (COUPONS.md); "
         "no Oldham since D26", STUB[1]),
        ("Fasteners (D26)", "DIN 7984 M3 × 6 8.8, 100-pack", 1, *M3X6_PACK[:1], "8 per drive stub: the Step-1 stub now, the arm's others in Step 2", M3X6_PACK[1]),
        ("Thermal (M7)", "10 × 10 k NTC thermistors (epoxy bead) + Kapton tape", 1, 6.0, "motor case and winding-side temperatures from day 1 (thermal.md)", E),
        ("Fasteners (coupons)", "M4 brass heat-set inserts, **short** (Ø5.6 hole × 5.7 mm), 100-pack", 1, 15.93,
         "coupons first (insert pull-out, Fable C1/C3), then the arm", "Newark $15.93 / 100 (M4 × 8.1 Heat-Sert class; short variant ESTIMATE)"),
        ("Fasteners (coupons)", "M3 brass heat-set inserts (Ø4.0–4.1 hole), 100-pack", 1, 12.0, "coupons, then the arm", E),
        ("Fasteners (coupons)", "Stainless screw kit: M4 × 12 countersunk + button heads (Fable minor), M4 / M3 SHCS assortment, M5 × 30 + nylocs",
         1, 25.0, "Step-1 rig, coupons, later the cartridge flanges", E),
        ("Consumables", "Epoxy (CF bonds), grease, thread locker 243 / 270, PTFE washers", 1, 15.0, "", E),
        ("Filament (A6)", "PETG 3 kg + ASA 1 kg (coupons, a sleeve coupon, the first cartridge can + cap) + TPU 95A 0.25 kg", 1, 90.0,
         "coupons now, the arm in Step 2 (print_plan.md)", E),
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
        ("Joints (C1, W8c)", "Steel crush sleeves, turned 42CrMo4+QT: pitch tie-rods 8 × Ø7 / 4.3 (4 × 49.75 + 4 × 19.75 mm) + yaw flange 6 × Ø4.6 / 3.2 × 29.65 mm "
         "(ends faced, length = the measured print − 0.05)", 14, *SLEEVE[:1],
         "Neon #2 F3: printed clamps kept ≈ 30 % preload; sleeved 88 / 87 % over life (rnd/joints.py); no Belleville stacks", SLEEVE[1]),
        ("Structure (W8b)", "D19 elbow shaft core: 6061-T6 tube Ø30 × 1.5, one piece cut to 139.5 mm", 1, 10.0,
         "Neon #2 F7: the bonded core takes ≈ 85 % of the bending (print 5.0 / 14 → 0.7 / 2.0 MPa)", "ESTIMATE: 0.25 m of a metric 6061 tube (" + ELBOW_TUBE[1].split("(", 1)[1]),
        ("Consumables (W8b)", "Structural epoxy (Loctite EA 9466 / 3M DP420 class, 50 ml + nozzles)", 1, *EPOXY[:1], "elbow core tube bond; the CF tube bonds", EPOXY[1]),
        ("Fasteners (W8b)", "Flange ears, 3 GIM6010 stages × 4: 10 × ISO 4762 M4 × 12 + 2 × M4 × 16 (pitch, reversed from the pulley side), 12 × ISO 4032 M4 nuts, "
         "12 × ISO 7089 washers", 1, 4.0, "Neon #2 F2: 4 ears per site (yaw side-fed nuts, pitch4 rear pair reversed); tipping ≥ 10× at 30 % preload", E),
        ("Fasteners", "Hardened dowels Ø6 × 10 (yaw hard-stop pin) + Ø6 × 12 (3 × KX-V ferrule dowels)", 1, 6.0, "yaw stops ±155°, arm detach ferrule", E),
        ("Structure", "Carbon tubes Ø40×1 + Ø30×1 (one arm)", 1, 40.0, "upper-arm neck (bond ≈ 55 mm), forearm link (30 mm into each socket)", E),
        ("Structure", '3" quick-release squeeze tri-clamp, 304 SS', 1, 25.0, "KX-V ferrule at the shoulder (arm detach)", "hosewarehouse (Dixon 13MHHM-Q300)"),
        ("Balancer (A9)", "ISO 10243 green die spring Ø25 / Ø12.5 × L0 115, 18.7 N/mm (Berger Tools V25-115 or Accu TJL25-115) + 2 mm UHMWPE cord", 1, 14.0,
         "2:1 tackle balancer, pin B b 20 mm, a 40–60 mm (balancer.md)", E),
        ("Sensors (M3)", "4 × AS5048A joint encoders + 2 RP2040 CAN nodes (shoulder + elbow) + 2 tactile pads + palm ToF", 1, 96.0,
         "joint-side encoders; **no mount or node firmware yet** (Fable M3): buy with the arm, or drop the ≤ 1 mm accuracy claim for M1", E),
        ("Actuators (from Step 1)", "Feetech STS3215-C018 **12 V** (30 kg·cm)", 2, 19.5, "wrist roll (D16 spindle), gripper: on the elbow node's 12 V 6 A buck (below), "
         "which Step 1 did not have (Neon #2 F15: the 1 A logic buck must never feed them)", "loop 14 retailer snapshot"),
        ("Electronics (from Step 1)", "USB TTL bus-servo adapter (STS3215 ID setup only)", 1, 8.0, "servo IDs 1 / 2", E),
        ("Yaw stage (D24 v2, from Step 1)", "Aluminium HTD 5M 20T, 12 H7 + 4 mm keyway", 1, *KEYED_PULLEY[:1], "yaw small pulley", KEYED_PULLEY[1]),
        ("Yaw stage (D24 v2, from Step 1)", "DIN 6885 4 × 4 key assortment (16 mm)", 1, *KEYS[:1], "one under each stage's pulley (yaw, pitch, elbow)", KEYS[1]),
        ("Yaw stage (D24 v2, from Step 1)", "HTD 5M-15 closed belt 315 mm (yaw, 20T:40T) at the **80.93 mm** exact centre", 1, *BELT[:1],
         "belts.md: the CAD's 80.00 mm uses 93 % of the ±1 mm adjuster; the CAD track moves it before printing", BELT[1]),
        ("Yaw stage (D24 v2, from Step 1)", "61808 (40×52×7) × 2", 1, 2 * B61808[0], "yaw one-piece shaft (D19)", B61808[1]),
        ("Stages (D26)", "6001-2RS (12×28×8): the stub's far end in wall B's bonded bushing", 3, *B6001[:1], "yaw, pitch, elbow (the 6801 pairs went with the Oldham shafts)", B6001[1]),
        ("Stages (D26)", "Pitch idler: 2 × 625-2RS + M5 axle, captive nut, spacers, M4 jack screw", 1, 2 * B625[0] + IDLER_HW[0],
         "the pitch belt's back-side slotted idler (rnd/belts.py IDLERS); elbow / yaw run at fixed centres", B625[1] + "; " + IDLER_HW[1]),
        ("Stages (D26)", "Pitch exit-slot strap, 3 mm 6061-T6, + 4 × M4 × 12 + 2 inserts", 1, STRAP[0] + STRAP_HW[0],
         "ties the inner wall across the cartridge's exit slot (shoulder.slot_strap; the printed key is in the print plan)", STRAP[1] + "; " + STRAP_HW[1]),
        ("Thermal (M7, from Step 1)", "Aluminium finned rear cap for the yaw GIM6010 cartridge, **after seller question 6** confirms the mounting face", 1, *AL_CAP[:1],
         "sleeve vs cap difference (thermal.md: closed sleeve 0.65 × rated continuous)", AL_CAP[1]),
        ("Sensors (D14)", "Elbow node power + bus: 24→12 V **6 A** buck (two stalled STS3215 draw 5.4 A) + half-duplex TTL transceiver + 2 × MCP2562", 1, 14.0,
         "D14: 12 V and TTL made in the arm (sw/config/can_map.yaml)", E),
        ("Sensors", "Global-shutter wrist camera (OV9281 MIPI-CSI) + CSI→USB UVC bridge + slim high-flex USB cable", 1, 70.0, "D12 wrist camera", E),
        ("Harness", "Clock-spring ribbon stock: 16 AWG high-flex ETFE/FEP pair (Neon #2 F10; was 20 AWG silicone), 26 AWG twisted pairs, shielded slim USB, Kapton/PET carrier + PTFE tape; JST-GH/VH kit", 1, 24.0,
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
         ("Drive (D20, D26)", "Turned C45 drive stub for the GIM8108-8 output (8 × Ø3.4 on its PCD; no Oldham since D26)", 2, STUB[0],
          "pitch + elbow (cartridge_m.parts_list_direct(MOTOR_8108): 6001 2.0×, stub 8 × M3 1.7× at the 18 N·m flash limit)", STUB[1]),
         ("Drive (D20, D26)", "DIN 7984 M3 × 6 **10.9**, 50-pack", 1, M3X6_109[0], "8 per D20 stub (the 8.8 pack serves the GIM6010 stubs)", M3X6_109[1]),
         ("Drive (D20, W8b)", "Steel C45 HTD 5M 20T, 15 mm belt, 12 H7 + 4 mm keyway (key 4 × 4 × 20; 6001s from the common line)", 2, 18.0,
          "the direct stage's 20T (belt_stage.AL_PULLEY note: steel 20T for 22 N·m)", E),
         ("Drive (D20, W8b)", "HTD 5M 15 mm **PU steel-cord** closed belts 355-5M (pitch) + 380-5M (elbow)", 2, 15.0,
          "cartridge_m.parts_list(MOTOR_8108) (Fable M6: a rubber 15 mm belt is 2.8× over ultimate/3 at 22 N·m)", E),
         ("Thermal (M7)", "Aluminium finned rear caps, Ø96 class", 2, 18.0, "pitch + elbow GIM8108-8 cartridges", E)],
    no=[("Actuators", "SteadyWin GIM6010-8 24 V SDC104", 2, GIM6010[0], "pitch + elbow as today (bench-valid at 1 kg; 0.6–0.8× on the 20 lb hug)", GIM6010[1]),
        ("Drive (D26)", "Turned C45 drive stub for the GIM6010-8 output, 8 × DIN 7984 M3 × 6 from the Step-1 pack", 2, STUB[0], "pitch + elbow (no Oldham since D26)", STUB[1]),
        ("Drive (D24 v2)", "Aluminium HTD 5M 20T, 15 mm belt, 12 H7 + 4 mm keyway (keys from the common assortment)", 2, KEYED_PULLEY[0], "pitch + elbow small pulleys", KEYED_PULLEY[1]),
        ("Drive", "HTD 5M-15 closed belts 355 (pitch, 20T:60T) + 295 (elbow, 20T:40T) mm", 2, BELT[0], "pitch + elbow", BELT[1]),
        ("Thermal (M7)", "Aluminium finned rear caps, Ø86 class", 2, AL_CAP[0], "pitch + elbow GIM6010 cartridges", AL_CAP[1])],
    note="Fable M6: the D20 swap is a drive redesign, not a motor swap. The cartridge track's D20 list (cartridge_m.parts_list_direct(MOTOR_8108), D26) "
         "puts the steel 20T on a turned C45 stub bolted to the GIM8108-8 output with 8 × M3 × 6 10.9 (1.7× at the 18 N·m flash limit), a 6001 in wall B "
         "(2.0×), PU steel-cord 15 mm belts; the D63 Oldham sets and doweled hubs of W8b went with the couplings (D26). "
         "The 24 V question is the other gate: most GIM8108-8 listings are 48 V nominal.",
    hi=lambda: 2 * (GIM8108_8_HI - GIM8108_8[0]) + line_sum(HOLD_A["yes"]) - line_sum(HOLD_A["no"]),
)

# Hold B (D24): the inline roll is the plan (owner OK 2026-11-27); the belt roll is the fallback if the drawing or the
# protocol fails.
HOLD_B = dict(
    title="Hold B: inline GIM8108-36 at shoulder roll (D24, **approved by the owner 2026-11-27**)",
    waits="the seller's answers (output face, output-bearing radial rating ≥ 612 N and tilting moment, backlash, SDC303 protocol and CAN timeout) and "
          "the Step-1 protocol result; the *no* list is the fallback (belt roll)",
    yes=[("Actuators", "SteadyWin GIM8108-36, **SDC303 (24 V)** driver", 1, GIM8108_36[0], "shoulder roll, inline 36:1", GIM8108_36[1]),
         ("Bearings", "61806 (30×42×7)", 1, 3.0, "rear roll support on the roll link's stub (68 mm from the motor's output bearing)", E),
         ("Drive (D24 inline)", "Output adapter, 7075-T6 Ø76 × 7 (CNC: Ø30 pilot, 3 × Ø4 H7 reamed, 3 × M4 csk, 8 × M5 tapped on PCD 64)", 1, 30.0,
          "motor Ø30 boss → printed clevis arm; torque on the 3 dowels (1.6× stall, rnd/joints.py on the C1/M1 track)",
          "ESTIMATE: JLCCNC / PCBWay-class instant quote ≈ $20–25 each at qty 2 + shipping share"),
         ("Drive (D24 inline)", "Ø80/Ø52 × 2 steel clamp ring + 8 × M5×16 DIN 7984 10.9, 3 × M4×10 ISO 10642, 3 × Ø4 m6 dowels", 1, 9.0,
          "adapter ↔ clevis arm clamp, boss ↔ adapter", E),
         ("Drive (D24 inline, W8c / D26)", "Crush sleeves from stock E235+C precision tube (D26; were turned 42CrMo4 at $7 each): clevis-arm M5 8 × Ø8 / 5.6 (8 × 1.2 tube) "
          "+ rear mount 6 × Ø6 / 4.4 (6 × 0.8) × 18.95 mm, cut and faced by the builder to the measured print − 0.05 ± 0.02", 14, 0.0,
          "Neon #2 F3/F6: the 8.5 mm PETG clevis clamp kept ≈ 12 % of its preload; the rear stack is metal (plate + sleeves + motor), 70 % left after embedding; "
          "sleeve yield 1.69× (rear) / 2.34× (clevis) at 235 MPa (rnd/joints.py)", "material in the tube line below"),
         ("Drive (D24 inline, W8c / D26)", "E235+C seamless precision tube, 500 mm sticks: 8 × 1.2 + 6 × 0.8 (the 28 clevis + rear sleeves of both arms)", 2, SLEEVE_TUBE[0],
          "D26: stock tube where its yield clears the sleeve stress; pitch / yaw sleeves stay turned (rnd/joints.py SLEEVE_STOCK)", SLEEVE_TUBE[1]),
         ("Drive (D24 inline, W8c / D26)", "Sleeve facing jig: mini tube cutter, abrasive on a flat plate, drilled guide blocks Ø6 / Ø8", 1, FACING_JIG[0],
          "every sleeve to the measured print − 0.05 ± 0.02 (≈ 3 min each, 28 per robot)", FACING_JIG[1]),
         ("Drive (D24 inline, W8c)", "Stepped hard-stop bushes, 42CrMo4+QT (Ø10 × 8 proud / Ø7 tail), on 2 of the 8 rear M4", 2, STOP_BUSH_P[0],
          "Neon #2 F6d: the Ø10 stop pins had no seat wall; bush → plate → screws → housing closes in metal (1.81×)", STOP_BUSH_P[1]),
         ("Drive (D24 inline, W8c)", "Rear plate ring, 7075-T6 3 mm, Ø96 / 70", 1, PLATE_RING[0], "all-metal rear clamp (engagement 2.5–3.5 mm, no shim picking)", PLATE_RING[1]),
         ("Drive (D24 inline, W8c)", "Rear screws 6 × M4 × 25 ISO 10642 10.9 + 2 × M4 × 25 ISO 4762 12.9 (in the stop bushes) + 1 × DIN 471 Ø30 ring (61806 on the stub)",
          1, 5.0, "replace 8 × M4 × 18 + washers + DIN 988 shims and the 2 × Ø10 × 28 stop dowels", E),
         ("Drive (D24 inline, W8c)", "Loctite 660 (61806 outer race, 0.3 mm gap) + Loctite 638 (output dowels into Ø4.1 adapter holes)", 1, LOCTITE[0],
          "the 61806 floats ± 0.3 mm (forced load ≤ 800 N); dowels bonded seated in the motor's holes (150 MPa with load sharing)", LOCTITE[1])],
    no=[("Actuators", "SteadyWin GIM6010-8 24 V SDC104", 1, GIM6010[0], "shoulder roll on the 18T:80T belt (4.44:1)", GIM6010[1]),
        ("Drive (D24 v2)", "Aluminium HTD 5M 18T, 15 mm, 12 H7 + 4 mm keyway (+ keys)", 1, KEYED_PULLEY[0] + 2.5, "roll small pulley", KEYED_PULLEY[1]),
        ("Drive (D26)", "Turned C45 drive stub + 6001-2RS (direct stage, no Oldham)", 1, STUB[0] + B6001[0], "roll stage", STUB[1] + "; 6001: " + B6001[1]),
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
    "**GIM8108-8, 24 V:** we run a 24 V bus (6S Li-ion, 25.2 V full; our regen clamp switches on at 26.4 V and holds it ≤ 27.3 V, absolute max 28 V). Which driver "
    "version of the GIM8108-8 (SDC104, GDZ468, GDS68?) do you ship, and is it specified for 24 V? Please give rated torque, rated speed, "
    "no-load speed and continuous current at 24 V (listings quote 48 V). Price and lead time for 2 now (4 later).",
    "**GIM8108-8 drawing:** installation drawing with frame diameter and height, output-flange bolt pattern (hole count, PCD, thread, depth) "
    "and housing / rear mounting pattern, and the connector position.",
    "**GIM8108-36 (SDC303, 24 V):** which face is the output; the output flange material and the dowel-hole depth; reducer **backlash** "
    "(listings say 15 arcmin: measured how?); frame and connector position (we have the Ø96 × 57 drawing).",
    "**GIM8108-36 output bearing:** bearing type and allowable radial load at the output flange: we need **≈ 612 N radial in service** (W8c, 2026-11-27: the external 61806 now floats axially, so the output bearing carries more; was ≈ 280 N), "
    "and a **tilting-moment rating** (≥ 137 N·m would let us delete the 61806 support); also the allowable axial load and tilting moment (one listing gives 1000 N "
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
    "**D26 cost-down (owner, 2026-11-27): the Oldham couplings go.** Each GIM6010 / GIM8108-8 stage drives its 20T from a turned C45 stub bolted to "
    "the motor output (8 × M3 × 6), the stub's far end in a 6001-2RS in wall B (the lift: a 61901); the pitch belt is tensioned by a slotted idler "
    "(2 × 625-2RS, M5 axle, M4 jack screw), elbow / yaw at fixed centres. Out: the D50 sets + 6061 motor hubs (Step 1 and hold A / B *no*), the D63 "
    "sets + doweled hubs (hold A *yes*), the keyed Ø12 rod and the 6801 pairs; in: one stub per stage, 6001s, the pitch idler and exit-slot strap.",
    "**W10 BOM consolidation (2026-11-27; W8b cartridge, W8c load path):** the D50 Oldham is priced as sourced parts (Ruland MOCC51-12-A keyed clamp hub "
    "$67.19 + OD32/51-AT disc $36.45, Zoro) instead of a $20 generic ESTIMATE, plus a 6061 motor-side hub (the printed ASA tongue was rejected: "
    "0.74× friction at the peak) and DIN 7984 M3 × 6 screws; Step 1 has the one set + hub for its GIM6010, hold A *no* / hold B *no* the others, "
    "hold A *yes* D63 sets (≥ 25 N·m, M5 clamp, doweled hubs) on the shared Ø12 keyed stage with steel 20T and PU steel-cord belts "
    "(cartridge_m.parts_list(MOTOR_8108)). Step 2 adds the C1 crush sleeves (pitch 8, yaw 6), the 6061 elbow core tube + structural epoxy and the "
    "4-ear flange screws; hold B *yes* the inline roll's clevis + rear sleeves, 2 steel stop bushes, the 7075 rear plate ring, M4 × 25 rear screws, a "
    "DIN 471 ring and Loctite 660 / 638 (the M4 × 18 + shims and the Ø10 stop dowels go). No Belleville springs.",
    "**Neon review #2 finding 15 (2026-11-27):** Step 1 no longer buys the D40 Oldham, plain Ø12 rod, set-screw pulley, 315 belt and stage bearings "
    "(belts.md: D40 disc 0.86×, plain clamp hub 0.65× at the 11 N·m peak; the yaw centre uses 93 % of its adjuster) nor the CNC rear cap (seller question 6 "
    "is open) nor the two STS3215 + USB-TTL (the only 12 V in Step 1 would have been the safety board's 1 A logic buck): the GIM is tested on a lever bolted "
    "to its output, one keyed D50 Oldham stays for the tongue coupon, and the keyed yaw stage, the cap and the servos are Step 2 lines. The safety board "
    "follows netlist v3 (Neon #2 F1: NH-50 + freewheel diodes, TLV3202, K1 hold on AUX, 60 A main fuse) and is ordered after its schematic review.",
    "**Two steps (Fable verdict):** Step 1 is one GIM6010-8, the DM-J4310, the two STS3215, the CAN adapters, the bench supply, the whole "
    "safety board, one D13 stage on a test lever, thermistors + one aluminium cap, inserts / screws for the coupons and filament (trimmed by Neon #2 F15, first item). Everything "
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
    "Aluminium HTD 5M pulleys (Step 2): 15 mm belt, **12 H7 bore with a 4 mm keyway**, set screw on the key.",
    "Drive stub (D26): measure the GIM output's bolt circle, pilot and output-bearing position on the Step-1 motor (coupon_motor) and set belt_stage.STUB / "
    "MOTOR['out_pcd'] before quoting the stubs from the STEP; the 6001 seat k6, the pulley seat h6 with a 4 mm keyway.",
    "Crush sleeves (W8c, D26): measure each printed stack; pitch / yaw sleeves turned 42CrMo4+QT (R_p0.2 ≥ 750 MPa on the certificate) to print − 0.05 mm; clevis / rear sleeves cut from the E235+C tube sticks and faced in the jig to print − 0.05 ± 0.02 (micrometer).",
    "Vishay NH-50 (not RH-50) for the three clamp resistors: the part number starts NH050; the pre-charge and dummy-load resistors stay RH-50.",
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
         "contactor, watchdog at 25 ms, e-stop → no torque ≤ 50 ms) with a real drive; torque constant and friction on a lever bolted to the output; the collision "
         "trip on one joint; the motor case temperature rise at rated torque on the lever; insert pull-out, the bolted-flange coupon and the drive-stub coupon (C45 stub on the GIM output, 8 × M3 × 6).", "",
         '**Step-1 exit criterion added 2026-11-27 (software track: determinism, the Asimov sim2real lessons): soak test.** `python -m sw.soak --joints yaw,wpitch --hours 2` on the Step-1 rig. A sysid fit at the start (bring-up step `sysid`, sw/sysid.py: Coulomb + viscous friction, effective torque constant, backlash, command → response latency → sw/config/actuators_<serial>.yaml), then 2 h of 0.25 Hz cyclic motion at ≈ 50 % rated torque (2 kg at the 0.3 m lever end on the yaw stage, 0.5 kg at the wrist), then the sysid fit again with the drives hot. **Pass:** Coulomb drift ≤ 20 %, viscous ≤ 25 %, torque constant ≤ 5 %, backlash growth ≤ 0.1°, latency mean ≤ +0.5 ms and p95 ≤ +2 ms; no supervisor stop, no CAN bus-off, no new Pi throttle / under-voltage flag (`vcgencmd get_throttled`), zero implausible (corrupted) CAN frames, loop jitter p99 < 1 ms, CPU < 80 °C (rehearsed on the virtual bench: a stable drive passes, a drive whose friction drifts with temperature fails; docs/sw/sim2real_bench.md). The lever measurements of the torque constant and friction above are this step. **Proposed Step-1 line (lead to add in rnd/m1_kit.py):** one Waveshare 2-CH CAN FD HAT ($46.99) to measure SPI-CAN (MCP2518FD) against the two gs_usb adapters on the same bus ([can_interface_note.md](sw/can_interface_note.md)).', "",
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
          "GIM6010-8 → bring-up on the Step-1 rig (`--joints yaw,wpitch`) → coupons (insert pull-out, bolted flange + drive-stub coupon: "
          "[COUPONS.md](COUPONS.md)) → Step 2. Readiness and the go / no-go per step: [M1_READINESS.md](M1_READINESS.md). Buying is the owner's decision."]
    (DOCS / "M1_BENCH_ARM.md").write_text("\n".join(L) + "\n")
    print("\n".join(L[2:10]))
    return t


if __name__ == "__main__":
    main()
