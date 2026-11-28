"""Milestone M1: one-arm bench kit, bought in steps (Fable review 2026-11-27: two steps; Fable review #2 2026-11-28: Step 1 split).
python -m rnd.m1_kit

- **Step 0 (free, before any money):** read the public GIM6010-8 manual (rnd/specs/actuators.yaml, GIM6010-8 sources) and send
  the six seller questions (SELLER_QUESTIONS). Either can change Step 1a.
- **Step 1a (buy now): protocol discovery on a bare, unloaded GIM at a 1 A limit; no safety board.** One GIM6010-8 (the
  yaw motor later), the DM-J4310, the two CAN adapters and the CAN HAT, the 10 A CC supply, the stand and the lever stock,
  thermistors, inserts, screws, filament, and the coupon tools (C12 insert pull-out, C13 4-ear flange, C14 drive stub:
  the GIM output's tilt stiffness, PCD and pilot, and the 8 × M3 joint on a dummy output). Fable #2 verdict.
- **Step 1b (later): the loaded rig and the safety chain.** The SE-1000-24, every safety-board part, the motorless test
  kit and its instruments, the integral pinion (quoted after C14) and the rig's loaded parts (W17a: second lever, steel
  discs, rubber stops, polycarbonate guard, interlock switch; docs/M1_RIG.md §5). The rig's stand, plate stock, tilt
  mandrel, indicators and scale are 1a lines. The category-0 catch moved to Step 2 (W17a: the pendulum rig cannot fall).
  Bought only when the 1a → 1b gate is met (STEP1B_GATE): discovery done, the board drawn in KiCad and reviewed (G-SB)
  with the logic race fixed (netlist v4, W17b), the rig designed (W17a), C14 measured. The G-SB review itself is an
  optional service line (SERVICES), outside every parts total.
- Old Step 1 (before the split): one GIM6010-8, the DM-J4310,
  the CAN adapters, the bench supply and the whole safety board, a test lever bolted straight to the GIM output, the
  joint's C45+QT integral pinion shaft (D26 v5: no Oldham, no keyed pulley) for the stub coupon, the motorless test kit for
  the board tests T0–T16, thermistors, inserts and screws for the coupons,
  filament. Neon review #2 finding 15
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

import os
from pathlib import Path

DOCS = Path(__file__).resolve().parent.parent / "docs"
CONTINGENCY = 0.15            # A16: price drift, shipping and tax (review finding 18)
E = "ESTIMATE"
E_D20 = "ESTIMATE (D20 redesign: the cartridge track sets the exact parts)"

# --- sourced unit prices used in more than one list (search snapshots 2026-11-27 unless noted) -------------------------
# D29 #2 (cost_down.md, sourcing only): the robot and Step 2 buy at Robotics Center's $70 (SDC104, 24 V; backordered); the
# current aifitlab $90 listing stays the fallback. Step 1 buys its one unit now, so it is priced at the fallback (it must not
# wait on a backorder).
GIM6010 = (70.0, "Robotics Center of Silicon Valley $70 (SDC104 driver, 24 V; 'backordered, sourced to order', 2026-11-28 snapshot); "
                 "**fallback: aifitlab $90 with SDC104** (loop 14) if the backorder outlasts the buy")
GIM6010_FALLBACK = (90.0, "aifitlab $90 with SDC104 (loop 14; the D29 fallback: Robotics Center's $70 is backordered, 2026-11-28)")
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
STOP_BUSH_P = (10.0, "ESTIMATE: stepped Ø10 / Ø7 bush, Ø7.2 / Ø4.3 bores, 42CrMo4+QT, turned (basis as the sleeves)")   # W13; out since W16c (R17)
# W16c (R17, 2026-11-28, rnd/parts/roll.py FRONT_STOP): the roll hard stop moved under the front collar: two 7075 dogs machined
# on the output adapter (so the adapter blank grows 7 → 19.5 mm) against two blocks on a 3 mm 7075 ring on the motor's front
# face (8 × M4 × 6 ISO 10642 into the front-face holes; the collar takes 2 × M3 × 10 radial into the blocks). Out: the 2 stop
# bushes per arm, their 2 socket-head M4 × 25 (all 8 rear screws are now ISO 10642 with a sleeve), the W14b collar's 4 × M4 × 25.
ADAPTER_7075 = (38.0, "ESTIMATE: 7075-T6 Ø76 × 19.5 blank (was Ø76 × 7 at $30): 3-axis CNC with the two stop dogs (r 31–36.5, 25°) left standing "
                      "on its back face; JLCCNC / PCBWay-class instant quote ≈ $20–25 each at qty 2 for the 7 mm part + ≈ $8 for the thicker 7075 "
                      "blank and the pocketing round the dogs")
FRONT_STOP_RING = (35.0, "ESTIMATE ($30–40): 7075-T6 front stop ring Ø95 / Ø60.6 × 3 with two blocks (r 30.3–37.5, 34.5°) standing ≈ 7 mm on it, "
                         "8 × M4 csk + 4 × M3 tapped; one 10 mm 7075 plate part, instant-quote class (as the yaw flange / output adapter)")
FRONT_STOP_HW = (3.0, "ESTIMATE: 8 × ISO 10642 M4 × 6 10.9 (ring → motor front face) ≈ $0.25 + 2 × ISO 4762 M3 × 10 (collar → blocks) ≈ $0.10, "
                      "from 25–50 packs")
# Fable #2 (2026-11-28) coupon tools for Step 1a (C12–C14, docs/COUPONS.md)
# W17a reconciliation (2026-11-28): the rig's optical-lever line carries the laser + mirror (this line had them at ≈ $10), and the
# rig's 10 mm 6061 plate is ordered 60 mm longer so the C14 dummy output comes off it (the $8 offcut line had it at ≈ $3).
DTI = (25.0, "ESTIMATE: a 0.002 mm dial test indicator with magnetic base (≈ $25); the laser pointer + mirror that were in this line (≈ $10) "
             "are the rig's optical-lever line since W17a")
DUMMY_OUT = (5.0, "ESTIMATE: S-hook + 10 L bucket (the C12 / C14 loads); the 6061 60 × 60 × 10 offcut is cut from the rig's 10 mm plate since W17a (RIG_STOCK)")
# W17a Step-1 lever rig (docs/M1_RIG.md §5, rnd/parts/rig.py; prices ESTIMATE)
RIG_STOCK = (38.0, "ESTIMATE: W17a $35 for the 300 × 150 plate, Ø65 bar, 12 mm stop plate and 50 × 40 bar, + ≈ $3 for the 60 mm longer plate "
                   "(the C14 dummy output, formerly a separate offcut)")
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
AL_CAP_MACHINED = (15.0, "ESTIMATE: 6061 Ø86–96 × 6 disc with fins, JLCCNC / PCBWay-class instant quote")
# D29 #5 (PIP_REAR_CAP=stock, not adopted): the stock round LED heatsink at its US retail listings, not the $6 the cost-down
# list assumed (that is an Alibaba wholesale price with a minimum order)
AL_CAP_STOCK = (25.79, "Newegg 90 × 10 mm round 10 W LED aluminium heatsink: 5-pack $128.95 ($25.79 each), single $34.00–35.49, ships from "
                       "China (2026-11-28 search); Alibaba sunflower extrusions from ≈ $4.80/kg need a minimum order; + facing and 3 holes by the builder")
from .params import CASTER_RELEASE, DETACH_CLAMP, JOINT_ENCODER, REAR_CAP  # noqa: E402  (D29 switches)
AL_CAP = AL_CAP_STOCK if REAR_CAP == "stock" else AL_CAP_MACHINED
# D29 #7 (sourcing only, no design change): Vishay NH-50 0.56 Ω 1 % non-inductive, part number NH050R5600FE02 (inferred from
# Vishay's NH code: NH050R1000FC02 = 100 mΩ, NH05050R00FE02 = 50 Ω); no distributor listing of that value was found
NH50 = (18.0, "ESTIMATE $18 each for NH050R5600FE02: NH-50 single-unit listings IBS NH05050R00FE02 $15.01 (10 wk), Tedss NH-50 0.5 Ω $19.99, "
              "0.2 Ω $15 ($12 at 5+), Quest NH-50 50 Ω $15 at 1 / $6.5 at 28 (2026-11-28 search; no 0.56 Ω price published, was $30 ESTIMATE); "
              "the wirewound RH-50 RH050R5600FE05 is the accepted alternate only with the freewheel Schottky (W17b; Neon #2 F1 was the RH-50 without one)")
NH50_ALT = ("Vishay RH-50 0.56 Ω 1 % (RH050R5600FE05 class), each with the same freewheel Schottky across it: the **accepted alternate** "
            "(W17b, electrical.md regen-clamp table: drain 31.5 V ≤ 48 V, L/R ≈ 9 µs vs the 50 µs minimum on-time) if the NH-50 lead time bites")
CLAMP_PARTS = 37.0      # the regen-clamp line less its 3 NH-50: Schottkys, MOSFETs, driver, caps, thermal switch, plate (ESTIMATE)
# D29 #9 (PIP_CASTER_RELEASE=cable): one central solenoid pulling the 4 lock plates (rnd/tip.py cable_release: the JF-1039 is 0.84×)
SOLENOID_55N = (26.62, "eBay JF-1264B 24 V push-pull solenoid, 55 N, 10 mm (2026-11-28 search snapshot); rnd/tip.py: 1.85× / 1.06× the 4-cable pull")
CABLE_KIT = (12.0, "ESTIMATE: 4 × 0.3 m Ø1 7×7 stainless cable + PTFE liners (bike-brake class), crimp stops, a printed equaliser plate")
# D29 #10 arm-detach clamp (params PIP_DETACH_CLAMP)
DETACH_CLAMPS = {"dixon": (25.0, '3" quick-release squeeze tri-clamp, 304 SS (Dixon 13MHHM-Q300)', "hosewarehouse (Dixon 13MHHM-Q300)"),
                 "wingnut": (10.40, '3" single-hinge wing-nut tri-clamp, 304 SS (generic homebrew class; hand-tight, then a firm quarter turn)',
                             "Howdybrewer 3\" 304 tri-clamp $10.40 (2026-11-28 search); Craft Kettle 3\" single-hinge heavy-duty $7.28, Steel & O'Brien 13MHHM-3-304 $17.69 (Zoro); "
                             "no generic quick-release squeeze clamp was found below the Dixon (Steel & O'Brien 13MHHMQ-3-304 $46.39, Zoro)")}
DETACH_CLAMP_P = DETACH_CLAMPS[DETACH_CLAMP]
# W15b base channel (docs/rnd/electrical.md *Base channel*, netlist Q9 / U20–U27 / Q10 / R7 / R8; safety_board_sim.md §6):
# robot-only parts (the M1 bench has no base); ESTIMATE, DigiKey-class unit prices unless a source is named.
Q9_SWITCH = (9.0, "ESTIMATE (DigiKey-class): TI LM5069-1 hot-swap controller (latch-off) ≈ $4.5 + IRFB7530 ≈ $3 + 1 mΩ 2512 sense ≈ $0.5 + "
                  "soft-start / timer passives ≈ $1")
BASE_CLAMP_REST = 30.0  # the base clamp leg less its NH-50: IRFB7530, VS-40CPQ060-N3, TLV3202, 2 × LVC1G123, UCC27517, passives (ESTIMATE)
BASE_LOGIC = (8.0, "ESTIMATE (DigiKey-class): U22 SN74LVC1G123 ≈ $1 + C0G/RC, U26 comparator + 50 ms RC ≈ $1.5, U27 AQV252G-class photoMOS ≈ $4, "
                   "R8 1 kΩ 3 W ≈ $1")
NC_BLOCK = (6.0, "ESTIMATE: snap-on NC contact block for the 22 mm mushrooms (IDEC / Schneider ZB5-class generic ≈ $4–8)")
SB1_2POLE = (3.0, "ESTIMATE: SB1 ARM / reset pushbutton NO → 2-pole NO (SB1b resets U20), the price difference")
# The 2.0 s OFF-delay timer U20 (D32, owner 2026-11-28: board timers for now; electrical.md 'Cost (BOM owner)'): params-style switch PIP_BASE_TIMER,
# read here because only the BOM and the M1 docs use it (no CAD or analysis does). Default "board" PROVISIONALLY.
BASE_TIMERS = {
    "board": (8.0, "two independent board-level OFF-delay timers (an SN74LVC1G123 RC timer + an independent crystal-timed counter; "
                   "either one drops Q9's enable), cross-checked at every power-up, set 2.25 s, ± 20 %, **not a certified safety part**",
              "ESTIMATE ≈ $8 for both (electrical.md: board-level dual timer); cross-check = the supervisor compares both edges at ARM (T17)"),
    "g9sa": (501.38, "Omron G9SA-321-T075 AC/DC24 time-delay safety relay (dual-channel input, OFF-delay safety contacts 0.5–7.5 s, set 2.0 s), "
                     "a certified part, 13-week lead",
             "DigiKey $501.38 (13-week lead); RS UK £344.61 ex VAT (2026-11-28 search)"),
}
BASE_TIMER = os.environ.get("PIP_BASE_TIMER", "board")
assert BASE_TIMER in BASE_TIMERS, f"PIP_BASE_TIMER must be one of {list(BASE_TIMERS)}"
# Step-1 test kit (W15b, safety_board_tests.md T12): the 30 A step = the 20 A supply + the 22 mF bank at 32.5 V through 0.33 Ω
# and a second VS-40CPQ060-N3
VS40 = (2.0, "RS £37 / 25 (the clamp line's VS-40CPQ060-N3 snapshot)")
R033_50W = (6.0, "ESTIMATE: 0.33 Ω 50 W aluminium-housed resistor (RH-50 class is fine here: a ms bank dump, not the clamp)")
# D29 #6 joint encoders (params PIP_JOINT_ENCODER)
ENCODER_P = {"as5048a": (12.0, "ESTIMATE: AS5048A board"), "mt6701": (4.0, "MT6701 boards: OnBuy 10-pack £22.33 (≈ $2.9 each), eBay $2.80, Walmart $7.10 (2026-11-28 search)")}[JOINT_ENCODER]
# Waves 12–13 (Astra review #3, 2026-11-28): the parts the merged design added or changed.
PINION = (35.0, "ESTIMATE (docs/rnd/belts.md, Astra #3 m7): one turned + hobbed C45+QT integral pinion shaft (stub v5: Ø42 × 4 flange, neck, "
                "20T HTD 5M cut on the shaft, Ø15 collar, Ø12 k5 6001 seat) ≈ $35 each in 10s vs ≈ $25 for the v4 stub + keyed pulley + key; "
                "quote from the STEP (belt_stage.small_stage_v4)")
TENSION_RING = (8.0, "ESTIMATE: turned 6061-T6 ring, bore eccentric by 0.30 mm to the OD, 1.2 mm min wall, 36 index marks (belt_stage.TENSION_RING, "
                     "belts.md D26 tension ring); basis: JLCCNC CNC turning from $5.00 a part (jlccnc.com, 2026-11-27 search) + bar")
C45_RING = (8.0, "ESTIMATE: turned C45+QT clamp ring (EN 10083-2, R_e ≥ 490), 2 mm, holes drilled (joints.md C1 contact faces, Astra #3 m9); "
                 "JLCCNC turning from $5.00 a part + QT bar")
YAW_FLANGE_7075 = (30.0, "ESTIMATE: 7075-T6 machined yaw flange (hex spigot, 6 × M3 tapped at the hex corners; was 6061-T6 at $25: Astra #3 m9, "
                         "joints.md); 7075 plate premium over 6061 ≈ +$5 at qty 2 (instant-quote class)")
# Safety board netlist v3 additions (Astra #3 critical 3, rnd/safety_board_sim.py, electrical.md netlist): ESTIMATE, DigiKey-class unit prices
BOARD_V3 = (14.0, "ESTIMATE (DigiKey-class): Q4 100 V logic-level MOSFET IRL540N class ≈ $2 (was the 30 V IRLB8721), U16 LVC1G123 50 µs "
                  "min-on one-shot ≈ $1, U17 LVC1G86 XOR + RC + comparator ≈ $2, U18 TLV3012-class 12 V UVLO + reference ≈ $2.5, "
                  "U19 ACS724-30A Hall sensor ≈ $5.5 + RC + comparator ≈ $1")
# Safety board netlist v4 (W17b, Fable #2 M2: the logic race fixed; electrical.md netlist U28 / U29)
BOARD_V4 = (1.50, "ESTIMATE (DigiKey-class): U28 SN74LVC1G74 RUN latch ≈ $0.50, U29 74LVC1G17 Schmitt buffer (AUX debounce) ≈ $0.40, BAT54 ≈ $0.15, "
                  "C_db 47 nF C0G ≈ $0.30, R_c 100 kΩ + R_d 4.7 kΩ 1 % ≈ $0.15 (W17b: ≈ +$1.50 a board)")
ACS724 = (6.5, "ESTIMATE: ACS724LLCTR-30AB ≈ $5.5 (DigiKey-class) + RC + comparator (one per arm branch on the robot board)")
# R17 roll latch (PIP_ROLL_GUARD=latch, default off: an OPTION, not in any total): power_off.md option O2
ROLL_LATCH = (14.0, "ESTIMATE (docs/rnd/power_off.md O2): 24 V pull solenoid $6–9, Ø10 pin + 42CrMo4 guide sleeve $3, spring $0.5, "
                    "hall switch + magnet $2, FET + wiring $1; ≈ 45 g per arm (rnd/parts/roll.py LATCH)")
# Step-1 gate: motorless test equipment (docs/rnd/safety_board_tests.md, Equipment); instruments are tools the owner may own
SCOPE = (499.0, "Siglent SDS1104X-E (100 MHz, 4 ch, 1 GSa/s) $499 list at siglentna.com / Transcat (2026-11-28 search)")
DIFF_PROBE = (175.0, "Micsig DP10013 (100 MHz, ±1300 V): Woot $139.99 (sold out), Telonic £174.16 inc VAT (out of stock) (2026-11-28 search); ESTIMATE $175")   # dropped from the kit (Fable #2 minor: low-side drains, a ×10 probe + ground spring)
CLAMP_100A = (220.0, "Micsig CP2100A (DC–800 kHz, 10 / 100 A): Digitalzakka $219.99, SainSmart $259.99 (2026-11-28 search); meets the test "
                     "sheet's ≥ 500 kHz (W15b, safety_board_tests.md *Current-clamp bandwidth*)")
CLAMP_LOW = (70.0, "ESTIMATE: Hantek CC-65-class DC/AC clamp (K1 coil ≈ 0.4 A, supply lead ≤ 30 A)")
CC_20A = (216.99, "Newegg 0–32 V / 0–20 A switching bench supply $216.99 (2026-11-28 search; confirm CC/CV on the datasheet); Extech 382276 $529 at TestEquity")


# Services: priced so the owner can choose, never in a parts total. (group, item, $ low, $ high, why, source)
SERVICES = [
    ("Service (G-SB, optional)", "**Review gate G-SB, route A: hired EE review** of the safety-board KiCad schematic + layout, fixed scope (the whole "
     "checklist, the logic as a state machine with the v4 race fix, the power stage, the T0–T16 limits, a written verdict; "
     "docs/rnd/safety_board_tests.md *Review gate G-SB*)", 400.0, 1200.0,
     "1b gate 2: before any 1b board part is ordered. **Route B: two qualified maker-community reviewers, $0** (or a small honorarium), 2–4 weeks "
     "instead of 1–2. The owner chooses the route and the person (W17b)", "ESTIMATE (W17b: 4–8 h of a degree-level power-electronics EE)"),
]

# (group, item, qty, unit $, why / where, source)
J4310 = (150.0, "Robotics Center $150 in stock (2026-11-27); AIFIT $125 backordered")

# Fable review #2 (2026-11-28) verdict: the 1a → 1b gate. Each item is work, not money; 1b is bought when all four are met.
STEP1B_GATE = [
    "**Discovery done** (1a): `python -m sw.gim_discover` on the bare GIM6010-8 at 1 A answers an MIT / impedance mode; enable, zero-gain and a "
    "low-gain impedance hold work on the unloaded output; the drive's **CAN timeout** disables it when frames stop; a **torque limit saved to flash** "
    "reads back after a power cycle (or the manual / vendor tool names the registers). If there is no impedance mode, no CAN timeout or no flash "
    "limit: do not buy 1b; redesign the safety argument first (the board's timing budget is part of it).",
    "**Board drawn and reviewed with the logic race fixed** (electrical track, Fable #2 M2). **Race fixed on paper in netlist v4 (W17b):** the RUN "
    "latch U28 makes K2_DRIVE = LATCH ∧ ¬RUN ∧ ¬PRE_TIMEOUT, so an uncommanded K1 drop-out fires FORCE on its own edge (coil unplugged: far end "
    "< 10 V in 2.0 ms vs T7's 4 ms); AUX is debounced by U29 (t_stable 2.1–5.7 ms); the discrepancy window is 40 ms; the U10 terms are unified. "
    "**Still open:** the KiCad schematic in the tree, and the review gate G-SB (docs/rnd/safety_board_tests.md): route A a hired EE "
    "(≈ $400–1,200, the optional service line) or route B two qualified community reviewers ($0); the owner picks the route and the person.",
    "**Rig designed: delivered in W17a** (docs/M1_RIG.md, rnd/parts/rig.py, sw/config/rig_step1.yaml; Fable #2 C1): `--rig step1` in the "
    "software (ratio 1.0, rated 5 / peak 11 N·m, software range −10° … +90°, the weighed lever as the gravity model); natural-rubber stops at "
    "−15° (reference) and +100° at the centre of percussion; steel discs bolted through the bar at 0.27 m; an interlocked 3 mm polycarbonate "
    "guard ≥ 40 mm clear of every swept part; soak 4 / 2 discs (RMS 55 / 57 % of rated, band 45–65 %); rig rehearsals pass. Its parts are the "
    "1a / 1b rig lines below.",
    "**C14 measured** (1a, docs/COUPONS.md): the GIM output's tilt stiffness, bearing layout, PCD and pilot. Only then is the integral pinion quoted, "
    "as drawn (hinged output) or as the overhung variant (moment-rigid output; rnd/belts.py STUB_SUPPORT_RULE).",
]


def step1a():
    """Step 1a (Fable #2 verdict, buy now): protocol discovery on a bare, unloaded GIM at a 1 A limit; no safety board."""
    j = J4310
    return [
        ("Actuators", "SteadyWin GIM6010-8 **24 V, SDC104 driver version** (ONE)", 1, *GIM6010_FALLBACK[:1],
         "protocol discovery, enable / zero-gain / impedance, CAN timeout and flash limit on the bare output at 1 A; C9 / C14 coupons; later the upper-arm yaw (20T:40T)",
         GIM6010_FALLBACK[1] + "; take the $70 Robotics Center unit instead if it ships in time (−$20)"),
        ("Actuators", "Damiao DM-J4310-2EC **V1.1**", 1, *j[:1], "wrist pitch; its MIT protocol is known: the second motor on the Step-1 bus", j[1]),
        ("Electronics (A14)", "USB-CAN adapter, candleLight / gs_usb (FYSETC UCAN, STM32F072), onboard 120 Ω (jumper)", 2, 19.0,
         "can_map.yaml buses B1 + B2 = SocketCAN can0 / can1", "MatterHackers $19 (2026-11)"),
        ("Electronics (A14)", "Waveshare 2-CH CAN FD HAT (2 × MCP2518FD, isolated, SKU 17075): SPI-CAN alternative, measured "
         "against the two gs_usb adapters on the Step-1 bus (docs/sw/can_interface_note.md)", 1, 46.99,
         "decides the robot's 5 CAN channels (A14): latency, jitter, loss, bus-off on the Pi 5; check its INT GPIOs vs 17/27",
         "waveshare.com $46.99 (2026-11-27)"),
        ("Electronics", "Raspberry Pi 5 Active Cooler (official; heatsink + PWM fan)", 1, 5.0, "no thermal throttling under the 500 Hz loop (Menlo sim2real lesson: throttling quietly degraded actuator response)", "raspberrypi.com list price $5 (ESTIMATE until a retailer snapshot)"),
        ("Power (A5)", "Adjustable 0–30 V / 10 A CC/CV bench supply, set **24.0 V, 1 A** for 1a", 1, 65.71,
         "the only supply in 1a: discovery on the bare output at a 1 A current limit (sw/gim_discover.py); its output switch is the stop. "
         "Later the 1 A first power-up and T0 / T9", "Newegg generic 30 V 10 A CC/CV $65.71 (2026-11-27)"),
        ("Step-1 rig (W17a)", "Rig stand (docs/M1_RIG.md §1): 40×40 B-type slot-8 profile cut to 2 × 900, 2 × 172, 2 × 469 (2.88 m); 40×80 × 360 crossbar; "
         "10 corner brackets; M6 / M8 T-nuts and screws; 4 × M8 core-bore screws; **keeps the 2 bench clamps** of the old stand line", 1, 75.0,
         "the H-base: the GIM and the DM back to back, axes 370 mm up; the 1a motor fixture and the 1a-11 tilt anchor now, the loaded rig's frame in 1b. "
         "Replaces \"Bench stand: 2 × 40×40 × 400 + brackets + 2 bench clamps\" ($30; +$45)", "ESTIMATE (W17a, M1_RIG.md §5)"),
        ("Step-1 rig (W17a)", "6061-T6 stock: 10 mm plate **300 × 210** (2 motor plates 130 × 150 + the 60 × 60 C14 dummy output); Ø65 round bar 30 mm "
         "(2 hubs Ø64 × 10); 12 mm plate 60 × 270 (upper stop plate); 50 × 40 bar 200 mm (2 lower stop blocks 94 mm)", 1, *RIG_STOCK[:1],
         "drilled and tapped on a drill press; a lathe or a printed centring ring for the hub pilot. One cut order for the rig and the C14 dummy output", RIG_STOCK[1]),
        ("Step-1 rig (W17a)", "Tilt mandrel, 6061-T6 turned (rnd/out/rig/tilt_mandrel.step: Ø50 flange on the output's 8 × M3, Ø25 boss, Ø12 end) + 6001-2RS "
         "loading bearing at 97 mm", 1, 27.0, "1a-11 (= C14 (a)): the GIM output's tilt stiffness k_t and dead band under a 0–200 N radial load (Fable #2 M3)",
         "ESTIMATE: quote from the STEP (W17a)"),
        ("Step-1 rig tools (tools)", "2 × dial indicator 0.01 mm / 10 mm + 2 magnetic bases; 6 × 50 × 300 steel flat bar (the bases' reference, clamped to "
         "the motor plate)", 1, 46.0, "1a-11: the Ø25 boss read at 20 and 80 mm from the output face; skip what the owner has", "ESTIMATE (W17a)"),
        ("Step-1 rig (W17a)", "Optical lever: laser pointer + 10 mm mirror; M6 turnbuckle + Dyneema cord", 1, 13.0,
         "1a-11: 1 mm of spot = 0.25 mrad at 2.0 m (decides a moment-rigid output); the radial load through the luggage scale to an anchor on the stand's leg. "
         "Also the C14 optical lever (it was in the dial-test-indicator line)", "ESTIMATE (W17a)"),
        ("Step-1 rig tools (tools)", "Kitchen scale 5 kg / 1 g", 1, 12.0,
         "weigh every bar, hub, carrier plate, disc and bolt to ±1 g: the torque constant is fitted against the weighed lever (`kt_method: absolute`)", "ESTIMATE (W17a)"),
        ("Step-1 rig (W17a)", "Rig fasteners: 16 × M3 × 10 12.9 (hubs), 8 × M5 × 12 (bars), 8 × M6 × 35 + 8 Ø12 × 20 spacers (motor plates), 4 × M8 × 110, "
         "2 × M8 × 65 8.8 + nylocs (weight bolts), 4 × M10 nuts (bumper studs), 60 × M4 × 8 button heads (guard clips)", 1, 20.0,
         "the rig as drawn (rnd/parts/rig.py)", "ESTIMATE (W17a)"),
        ("Bench", "Lever stock: 6061 flat bar 40 × 6 × 300 mm (the GIM lever) + luggage scale (50 kg, peak hold); W17a: the 1 kg and 2 kg weights "
         "are out (the rig's steel discs replace them, −$12)", 1, 13.0,
         "1a: **unloaded** (no weight on the motor before the 1b rig, Fable #2 C1); the stub-flange stand-in for C14 (8 × M3 × 6 on the measured "
         "PCD), the C12 pull lever, the C13 torque pulls; the scale for the 1a-9 stall and the 1a-11 load; 1b: the GIM lever on the rig (docs/M1_RIG.md)", E),
        ("Thermal (M7)", "10 × 10 k NTC thermistors (epoxy bead) + Kapton tape", 1, 6.0,
         "motor case and winding-side temperatures (thermal.md); read with the DMM by hand in Step 1 (no ADC in the kit, Fable #2 minor)", E),
        ("Fasteners (coupons)", "M4 brass heat-set inserts, **short** (Ø5.6 hole × 5.7 mm), 100-pack", 1, 15.93,
         "C12 insert pull-out first (Fable C1 / #2 M4), then the arm", "Newark $15.93 / 100 (M4 × 8.1 Heat-Sert class; short variant ESTIMATE)"),
        ("Fasteners (coupons)", "M3 brass heat-set inserts (Ø4.0–4.1 hole), 100-pack", 1, 12.0, "C12, then the arm (rear-cap lugs, seat caps, bone cap)", E),
        ("Fasteners (coupons)", "Stainless screw kit: M4 × 12 SHCS + ISO 7089 washers + ISO 4032 nuts (the C13 ears), M4 × 12 countersunk + button heads "
         "(Fable minor), M4 / M3 SHCS assortment, M5 × 30 + nylocs", 1, 25.0, "C12 / C13 coupons, the 1a fixture, later the cartridge flanges", E),
        ("Fasteners (D26)", "DIN 7984 M3 × 6 8.8, 100-pack", 1, *M3X6_PACK[:1],
         "8 per drive stub: the C14 joint now (on the lever stand-in and the dummy output), the arm's stubs in Step 2", M3X6_PACK[1]),
        ("Consumables", "Epoxy (CF bonds), grease, thread locker 243 / 270, PTFE washers", 1, 15.0, "Loctite 243 on the C14 joint", E),
        ("Filament (A6)", "PETG 3 kg + ASA 1 kg (coupons C9 / C12–C14, the first cartridge can + cap) + TPU 95A 0.25 kg", 1, 90.0,
         "coupons now, the arm in Step 2 (print_plan.md)", E),
        ("Coupon tools (tools)", "0.002 mm dial test indicator with magnetic base (the laser + mirror are in the rig's optical-lever line)", 1, *DTI[:1],
         "C14 / 1a-11: the third indicator, on the motor housing rim (subtracts the plate's and the housing's own tilt), and the flange-rim reading "
         "at r ≈ 20 (Fable #2 M3: hinged or moment-rigid decides the pinion)", DTI[1]),
        ("Coupons (C12, C14)", "C14 dummy output (the 60 × 60 × 10 piece of the rig's 6061 plate, 8 × M3 drilled + tapped by the builder on the measured PCD); "
         "S-hook + bucket for the C12 / C14 loads",
         1, *DUMMY_OUT[:1], "the 8 × M3 × 6 + 243 joint to 1.5 × the 11 N·m peak without risking the motor (its own 11 N·m cannot hold 16.5)", DUMMY_OUT[1]),
    ]


def step1b():
    """Step 1b (Fable #2 verdict, bought after STEP1B_GATE): the loaded rig and the safety chain; its test kit is step1_test()."""
    return [
        ("Power (M2)", "**Mean Well SE-1000-24** (24 V 41.7 A, 1 kW), set **24.0 V** (not 25.2: the clamp's lowest turn-off is 25.44 V, Fable #2 minor)", 1, *PSU[:1],
         "arm 14.8 A sustained / 31.5 A peak at the rated drive limit; overload is a latched shutdown at 43.8–52 A, not current limiting (electrical.md *Bench supply*)", PSU[1]),
        ("Power (M2)", "4,700 µF 35 V bus capacitor + **60 A MIDI** main fuse (F1, sized for the robot pack) and 20 A ATO arm-lead fuse with holders + 10 / 14 AWG silicone wire, XT60",
         1, 30.0, "electrical.md safety-board netlist F1, F3, C1 (Neon #2 F1: a 40 A ATO never clears on the 41.7 A supply)", E),
        ("Power (A5 v2)", "Regen clamp / bus discharge: 3 × **Vishay NH-50 0.56 Ω (non-inductive)** + 3 × **VS-40CPQ060-N3 freewheel Schottky** + 3 × IRFB7530 + gate driver + gate resistors, film caps, 85 °C thermal switch, heatsink plate",
         1, 3 * NH50[0] + CLAMP_PARTS, "mandatory on a bench supply; Neon #2 F1: the wirewound RH-50 with no freewheel path avalanched the MOSFETs (electrical.md). "
         "**Accepted alternate:** " + NH50_ALT,
         "NH-50 (NH050R5600FE02) " + NH50[1] + "; VS-40CPQ060-N3 RS £37 / 25; rest ESTIMATE"),
        ("Power (A5 v2)", "E-stop: 2 two-channel latching mushrooms + TE ALEV100 DC contactor (24 V coil, aux contact) + K_PRE relay + 3.3 Ω RH-50 pre-charge "
         "+ SN74LVC1G123 watchdog on its own LDO", 1, 150.0, "category-0 stop; watchdog 25 ms (sized on sw/loop_jitter.py)",
         "LEV100A5ANG $92 (Master Electronics); rest ESTIMATE"),
        ("Power (M2)", "Safety-board logic (netlist v4 in electrical.md): LM74700-Q1 + IRFB7530 ideal diode, 2nd LVC1G123 (discharge one-shot on Ā), LVC1G74 latch + power-on reset, "
         "**TLV3202** + TL431B, LVC1G11 + LVC1G32 (K1 hold on AUX), LVC1G04, UCC27524, 12 V buck + 3.3 V LDO, 1 kΩ 3 W bleeder, 10 Ω RH-50 dummy load, 2 × PC817, 2-layer PCB; "
         "**v4 (W17b): U28 SN74LVC1G74 RUN latch, U29 74LVC1G17 AUX debounce + BAT54 + 47 nF C0G + 2 × 1 % resistors**", 1, 45.0 + BOARD_V4[0],
         "**order after the KiCad schematic is drawn and the review gate G-SB is closed** (1b gate 2; the race is fixed in netlist v4); board tests before any motor",
         "ESTIMATE $45; v4 parts " + BOARD_V4[1]),
        ("Power (netlist v3)", "Safety-board netlist v3 additions (Astra #3 critical 3, safety_board_sim.md): **Q4 100 V logic-level coil driver** (IRL540N class; "
         "the 30 V IRLB8721 saw ≈ 52 V at every turn-off), U16 50 µs minimum-on one-shot, **U17 AUX / K1 discrepancy monitor** (LVC1G86 XOR + 30 ms RC + comparator), "
         "**U18 12 V UVLO** (TLV3012 class, 10.5 V), **U19 arm-branch current trip** (ACS724-30A + 5 s RC + comparator at 16 A)", 1, *BOARD_V3[:1],
         "design changes 0–5 of the board simulation: no MHz ringing, no avalanche, welded K1 / stuck AUX / dead 12 V each end in a stop, the in-arm 16 AWG protected at 18–35 A", BOARD_V3[1]),
        ("Bring-up IO (S1)", "sw/bringup.py `--estop gpiod:gpiochip0:17:27`: reset/ARM pushbutton, coil MOSFET + suppression, opto "
         "(aux contact → GPIO27), heartbeat lead GPIO17 → watchdog, 3 m 4-core cable for the remote mushroom, terminal blocks", 1, 20.0,
         "safety step (e-stop, watchdog, reset refused while pressed); operator outside the fall envelope", E),
        ("Step-1 rig (W17a)", "Second 6061 flat bar 40 × 6 × 300 (the DM-J4310 lever)", 1, 8.0, "the wpitch lever on the rig's −Y face", "ESTIMATE (W17a)"),
        ("Step-1 rig (W17a)", "Weights: 8 × Ø60 × 10 S235 discs (Ø8.5 hole, 0.2175 kg) + 4 × Ø60 × 3 carrier plates, laser-cut or saw-cut bright bar", 1, 36.0,
         "bolted **through** the bar at r 270 (M8 × 65 + nyloc), in pairs; soak 4 discs on the GIM / 2 on the DM. Replaces the 1 kg and 2 kg weights "
         "of the 1a lever-stock line (−$12)", "ESTIMATE (W17a)"),
        ("Step-1 rig (W17a)", "4 × natural-rubber buffer 60 Sh A, Ø50 × 50, type D (one M10 stud)", 1, 12.0,
         "hard stops at −15° (reference) and +100°, struck by the disc rim at the centre of percussion; stiffness checked in 1b-0 (accept 90–360 kN/m)", "ESTIMATE (W17a)"),
        ("Step-1 rig (W17a)", "Polycarbonate 3 mm clear, cut: 2 × 626 × 597, 2 × 362 × 597 (2 notches each), 1 × 626 × 362 (cable slot) = 1.39 m²", 1, 70.0,
         "the lift-off guard, ≥ 40 mm outside every swept part; 15 × 60 slot 149 mm above the swept volume (ISO 13857); printed PETG clips and corners", "ESTIMATE (W17a)"),
        ("Step-1 rig (W17a)", "Magnetic coded safety switch, NC, with actuator", 1, 15.0,
         "the guard interlock, in series with the e-stop mushrooms: lifting the guard opens the contactor (check the sensing distance against the 5 mm riser gap)",
         "ESTIMATE (W17a)"),
        ("Drive (D26 v5)", "**C45+QT integral pinion shaft** for the Step-1 GIM6010 joint (stub v5, Astra #3 m7: Ø42 × 4 flange, 8 × Ø3.4 + c'bore on the "
         "measured output PCD, Ø16/10 neck, 20T HTD 5M hobbed on the shaft, Ø15 collar, Ø12 k5 seat for the wall-B 6001; no key, no pulley)", 1, *PINION[:1],
         "**quote only after C14** (Fable #2 M3: between the bearings or overhung depends on the output's tilt stiffness; a single piece may cost $100+, S); "
         "the yaw stage's pinion in Step 2", PINION[1]),
    ]


def step1():
    """Step 1 = 1a + 1b parts (the 1b test kit is step1_test()); kept for callers of the old single Step 1."""
    return step1a() + step1b()

def step1_test():
    """Step 1's gate equipment: what the motorless board tests T0–T16 need (docs/rnd/safety_board_tests.md, Equipment).
    Group 'Test kit' is consumed by the tests (bought); group 'Test instruments' are tools: skip any the owner already owns."""
    return [
        ("Test kit (T3–T8)", "Dummy drive bank: 5 × 470 µF 35 V low-ESR electrolytics (2.35 mF, the M1 drivers' input capacitance) on 0.5 m of 14 AWG + ring lugs, "
         "after the real 1.5 m arm lead and F3", 1, 8.0, "pre-charge, watchdog, release, forced-discharge and end-to-end stop tests at the drive end (TP2)", E),
        ("Test kit (T3–T12)", "Load resistors: 5.6 Ω and 2.2 Ω, 50 W aluminium-housed (the 10 Ω RH-50 pre-charge fault load is on the board line), on the "
         "clamp's heatsink plate", 2, 6.0, "the 2.2 Ω stands in for a drive in T8 / T16", E),
        ("Test kit (T11–T12)", "Regen-source diodes: 2 × VS-40CPQ060-N3 (2 × 20 A Schottky each, legs paralleled, on a small heatsink): one in series with "
         "the CC supply into the dummy bank, one switching the energy bank in for T12's 30 A step", 2, *VS40[:1],
         "safety_board_tests.md *Equipment* (W15b: 40 A Schottky, ≈ 15 W for 200 ms at 30 A; the 1N5822 is a 3 A part)", VS40[1]),
        ("Test kit (T12)", "0.33 Ω 50 W resistor: the energy bank's series resistor for T12's 30 A step (bank at 32.5 V + the 20 A supply)", 1, *R033_50W[:1],
         "W15b: the 20 A supply cannot make the 30 A step alone; the extra 10–15 A comes from the bank over ≈ 2.9 ms", R033_50W[1]),
        ("Test kit (T12)", "Energy bank 22 mF 50 V (2 × 10,000 µF + 2 × 1,000 µF 50 V, screw terminals) + a 10 A charge switch", 1, 18.0,
         "9.9 J at 30 V dumped into TP2 10× in 1 s (the pulse duty of the clamp resistors); charged to 32.5 V for the 30 A step", E),
        ("Test instruments (tools)", "4-channel oscilloscope ≥ 100 MHz, 1 GS/s (with its ×10 passive probes; add a ground spring per probe)", 1, *SCOPE[:1],
         "TP1–TP11: every pass / fail number is read on the scope, not in software. Fable #2 minor: no differential probe (−$175): Q1–Q3 and Q4 are "
         "low-side switches, so TP3 / TP7 drains are read to board ground with a ×10 probe and a ground spring (the SE-1000 output floats)", SCOPE[1]),
        ("Test instruments (tools)", "DC current clamp 100 A", 1, *CLAMP_100A[:1], "clamp leg 1 (49 A pulses), inrush ≤ 8.4 A", CLAMP_100A[1]),
        ("Test instruments (tools)", "Low-current DC clamp (≤ 30 A)", 1, *CLAMP_LOW[:1], "K1 coil current, supply lead", CLAMP_LOW[1]),
        ("Test instruments (tools)", "Second CC/CV supply 0–30 V / 20 A (the regen source; the 10 A Step-1 supply covers T0 / T9)", 1, *CC_20A[:1],
         "T10–T12: pushes current *into* the bus, which the SE-1000 cannot sink (≈ 500 W simulated regen)", CC_20A[1]),
        ("Test instruments (tools)", "DMM with a K-type thermocouple input + bead thermocouple", 1, 35.0, "rails (T0), resistor plate ≤ 60 °C (T12), wiring ≤ 60 °C (T16)", E),
        ("Test instruments (tools)", "Heartbeat source: the Pi's GPIO17 (a gpiod square at 250 Hz, stopped at a random phase) instead of a function generator", 1, 0.0,
         "T5 / T8 / T9 (the Pi drives GPIO17 on the robot anyway)", "owner has the Pi"),
    ]


def step2_common():
    """Step 2, independent of both holds: the rest of the arm."""
    return [
        ("Commissioning (A5 v2)", "Category-0 catch: adjustable padded cradle (Al bar + EVA foam) + Dyneema tether ≥ 600 N + payload tray", 1, 25.0,
         "drop energy ≤ 3 J with ≤ 50 mm gap (electrical.md); moved from Step 1b in W17a: the pendulum rig cannot fall, the cradle is for the arm", E),
        ("Bearings", "61808 (40×52×7)", 4, *B61808[:1], "pitch and elbow one-piece shafts (D19), 2 each", B61808[1]),
        ("Bearings", "6708 (40×50×6) thin", 2, 2.5, "elbow belt cassette", E),
        ("Bearings", "61804 (20×32×7)", 2, 0.92, "D16 wrist-roll spindle", "AliExpress MOCHU snapshot; Zoro $4.69"),
        ("Bearings (M8)", "608-2Z bearings (C0 1.37 kN) with printed POM tyres on a Ø8 axle fork", 2, 2.5, "A9 balancer pulley A + tackle pulley (Fable M8: a U604 carried 634–748 N on C0 ≈ 0.3 kN; M8 track 3220161)", E),
        ("Joints (C1/M1 fixes)", "Pitch-hub tie-rod: 4 × M4 class 10.9 studs (DIN 976-1) + ISO 4032 M4 nuts on a **C45+QT clamp ring** r 11–21 × 2 (was 6061-T6); "
         "yaw flange 6 × M3 × 40 ISO 4762 12.9 on a **C45+QT clamp ring** r 8.5–16.8 × 2 (was S235)", 1, 4.0 + 2 * C45_RING[0],
         "Fable C1 (the arm hung from M3 inserts in pull-out); Astra #3 m9 contact faces: nut on 6061 0.70×, sleeve on S235 0.64× → C45+QT 1.34–2.52× (joints.md)",
         "ESTIMATE: studs, nuts, screws ≈ $4; rings " + C45_RING[1]),
        ("Joints (C1/M1 fixes)", "Machined **7075-T6** yaw flange (hex spigot, 6 × M3 tapped at the hex corners; was 6061-T6)", 1, *YAW_FLANGE_7075[:1],
         "Astra #3 m9: the yaw sleeve feet on the spigot top, 6061 0.75× → 7075-T6 1.43× (joints.md)", YAW_FLANGE_7075[1]),
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
        ("Structure", DETACH_CLAMP_P[1], 1, DETACH_CLAMP_P[0], "KX-V ferrule at the shoulder (arm detach; D29 #10, PIP_DETACH_CLAMP)", DETACH_CLAMP_P[2]),
        ("Balancer (A9)", "ISO 10243 green die spring Ø25 / Ø12.5 × L0 115, 18.7 N/mm (Berger Tools V25-115 or Accu TJL25-115) + 2 mm UHMWPE cord", 1, 14.0,
         "2:1 tackle balancer, pin B b 20 mm, a 40–60 mm (balancer.md)", E),
        ("Sensors (M3)", "4 × AS5048A joint encoders + 2 RP2040 CAN nodes (shoulder + elbow) + 2 tactile pads + palm ToF", 1, 96.0,
         "joint-side encoders; the mounts are drawn (rnd/parts/encoder.py), the **RP2040 node firmware is not written** (Fable #2 M3): buy with the arm, "
         "or drop the ≤ 1 mm accuracy claim for M1", E),
        ("Actuators (from Step 1)", "Feetech STS3215-C018 **12 V** (30 kg·cm)", 2, 19.5, "wrist roll (D16 spindle), gripper: on the elbow node's 12 V 6 A buck (below), "
         "which Step 1 did not have (Neon #2 F15: the 1 A logic buck must never feed them)", "loop 14 retailer snapshot"),
        ("Electronics (from Step 1)", "USB TTL bus-servo adapter (STS3215 ID setup; the bench drive for the wrist roll + gripper from the Pi until the "
         "RP2040 node firmware exists, LeRobot-style)", 1, 8.0, "servo IDs 1 / 2; Fable #2 M3: without node firmware the servos move only through it", E),
        ("Yaw stage (D26 v5)", "HTD 5M-15 closed belt 315 mm (yaw, 20T:40T; the 20T is the Step-1 pinion shaft, so no pulley or key here)", 1, *BELT[:1],
         "belts.md: CAD centre 81.48 mm = stock belt 80.93 + the target stretch (D26_OFFSET), set by the tension ring", BELT[1]),
        ("Yaw stage (D26 v5)", "D26 tension ring, yaw (turned 6061-T6, e 0.30 mm) under the cartridge's centring lip", 1, *TENSION_RING[:1],
         "Astra #3 m6: fixed-centre preload 0–156 N → 134–220 N (0.77–1.27 × target), set by span frequency before the 6001 bond (belts.md)", TENSION_RING[1]),
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
         ("Drive (D20, D26 v5)", "C45+QT integral pinion shaft for the GIM8108-8 output (Ø18/10 neck, 20T hobbed, 8 × Ø3.4 on its PCD; replaces the v4 stub + "
          "steel keyed 20T)", 2, PINION[0],
          "pitch + elbow (belts.md stub table: D20 fatigue 0.68 → 2.63× at the 18 N·m limit; 6001 2.0×, 8 × M3 1.7×)", PINION[1]),
         ("Drive (D20, D26)", "DIN 7984 M3 × 6 **10.9**, 50-pack", 1, M3X6_109[0], "8 per D20 stub (the 8.8 pack serves the GIM6010 stubs)", M3X6_109[1]),
         ("Drive (D20, D26 v5)", "D26 tension ring, elbow (turned 6061-T6, e 0.30 mm; bore = the GIM8108-8 cartridge lip)", 1, TENSION_RING[0],
          "elbow fixed-centre stage, PU steel-cord 380-5M (belts.md)", TENSION_RING[1]),
         ("Drive (D20, W8b)", "HTD 5M 15 mm **PU steel-cord** closed belts 355-5M (pitch) + 380-5M (elbow)", 2, 15.0,
          "cartridge_m.parts_list(MOTOR_8108) (Fable M6: a rubber 15 mm belt is 2.8× over ultimate/3 at 22 N·m)", E),
         ("Thermal (M7)", "Aluminium finned rear caps, Ø96 class", 2, 18.0, "pitch + elbow GIM8108-8 cartridges", E)],
    no=[("Actuators", "SteadyWin GIM6010-8 24 V SDC104", 2, GIM6010[0], "pitch + elbow as today (bench-valid at 1 kg; 0.6–0.8× on the 20 lb hug)", GIM6010[1]),
        ("Drive (D26 v5)", "C45+QT integral pinion shaft for the GIM6010-8 output (20T hobbed), 8 × DIN 7984 M3 × 6 from the Step-1 pack", 2, PINION[0],
         "pitch + elbow (replaces the v4 stub + keyed aluminium 20T + key; belts.md fatigue 1.19 → 3.13×)", PINION[1]),
        ("Drive (D26 v5)", "D26 tension ring, elbow (turned 6061-T6, e 0.30 mm)", 1, TENSION_RING[0], "elbow 295-5M fixed centre: 0–165 N → 134–218 N (belts.md)", TENSION_RING[1]),
        ("Drive", "HTD 5M-15 closed belts 355 (pitch, 20T:60T) + 295 (elbow, 20T:40T) mm", 2, BELT[0], "pitch + elbow", BELT[1]),
        ("Thermal (M7)", "Aluminium finned rear caps, Ø86 class", 2, AL_CAP[0], "pitch + elbow GIM6010 cartridges", AL_CAP[1])],
    note="Fable M6: the D20 swap is a drive redesign, not a motor swap. The cartridge track's D20 list (cartridge_m.parts_list_direct(MOTOR_8108), D26) "
         "puts the 20T on a C45+QT integral pinion shaft (stub v5, Astra #3 m7: the keyed v4 stub was 0.68× in fatigue) bolted to the GIM8108-8 output with "
         "8 × M3 × 6 10.9 (1.7× at the 18 N·m flash limit), a 6001 in wall B (2.0×), PU steel-cord 15 mm belts, a D26 tension ring at the elbow; the D63 "
         "Oldham sets and doweled hubs of W8b went with the couplings (D26). "
         "The 24 V question is the other gate: most GIM8108-8 listings are 48 V nominal.",
    hi=lambda: 2 * (GIM8108_8_HI - GIM8108_8[0]) + line_sum(HOLD_A["yes"]) - line_sum(HOLD_A["no"]),
)

# Hold B (D24): the inline roll is the plan (owner OK 2026-11-27); the belt roll is the fallback if the drawing or the
# protocol fails.
HOLD_B = dict(
    title="Hold B: inline GIM8108-36 at shoulder roll (D24, **approved by the owner 2026-11-27**)",
    waits="the seller's answers (output face; the output bearing's type, radial, axial and tilting-moment ratings against ≈ 1,413 N radial in service / "
          "3,130 N at a fault, or 36.4 / 137 N·m tilting without the rear 61806; backlash, SDC303 protocol and CAN timeout) and "
          "the Step-1 protocol result; the *no* list is the fallback (belt roll)",
    yes=[("Actuators", "SteadyWin GIM8108-36, **SDC303 (24 V)** driver", 1, GIM8108_36[0], "shoulder roll, inline 36:1", GIM8108_36[1]),
         ("Bearings", "61806 (30×42×7)", 1, 3.0, "rear roll support on the roll link's stub (68 mm from the motor's output bearing)", E),
         ("Drive (D24 inline, W16c)", "Output adapter, 7075-T6 from a Ø76 × **19.5** blank (was × 7; CNC: Ø30 pilot, 3 × Ø4 H7 reamed, 3 × M4 csk, 8 × M5 tapped "
          "on PCD 64, **two R17 stop dogs** left standing on the back face)", 1, *ADAPTER_7075[:1],
          "motor Ø30 boss → printed clevis arm; torque on the 3 bonded dowels (1.03× stall, joints.md); W16c: the dogs meet the front stop ring's blocks "
          "(3.91× at stall, joints.md M1 hard stops)", ADAPTER_7075[1]),
         ("Drive (D24 inline, W16c)", "R17 front stop ring, 7075-T6 (3 mm ring + 2 blocks) on the motor's front face", 1, *FRONT_STOP_RING[:1],
          "W16c (rnd/parts/roll.py FRONT_STOP): the roll hard stop under the front collar, a couple (no motor-bearing load), metal to metal; replaces "
          "the 2 stepped stop bushes", FRONT_STOP_RING[1]),
         ("Drive (D24 inline, W16c)", "Front stop screws: 8 × ISO 10642 M4 × 6 10.9 (ring → motor front face) + 2 × ISO 4762 M3 × 10 (collar → blocks, radial)",
          1, *FRONT_STOP_HW[:1], "the W14b collar's 4 × M4 × 25 were too long for the 3.5 mm threads; those holes now take the ring", FRONT_STOP_HW[1]),
         ("Drive (D24 inline)", "Ø80/Ø52 × 2 steel clamp ring + 8 × M5×16 DIN 7984 10.9, 3 × M4×10 ISO 10642, 3 × Ø4 m6 dowels", 1, 9.0,
          "adapter ↔ clevis arm clamp, boss ↔ adapter", E),
         ("Drive (D24 inline, W8c / D26)", "Crush sleeves from stock E235+C precision tube (D26; were turned 42CrMo4 at $7 each): clevis-arm M5 8 × Ø8 / 5.6 (8 × 1.2 tube) "
          "+ rear mount **8** × Ø6 / 4.4 (6 × 0.8) × 18.95 mm (W16c: +2 where the stop bushes were), cut and faced by the builder to the measured print − 0.05 ± 0.02", 16, 0.0,
          "Neon #2 F3/F6: the 8.5 mm PETG clevis clamp kept ≈ 12 % of its preload; the rear stack is metal (plate + sleeves + motor), 70 % left after embedding; "
          "sleeve yield 1.69× (rear) / 2.34× (clevis) at 235 MPa (rnd/joints.py)", "material in the tube line below"),
         ("Drive (D24 inline, W8c / D26)", "E235+C seamless precision tube, 500 mm sticks: 8 × 1.2 + 6 × 0.8 (the 32 clevis + rear sleeves of both arms; the rear 16 × 18.95 = 303 mm)", 2, SLEEVE_TUBE[0],
          "D26: stock tube where its yield clears the sleeve stress; pitch / yaw sleeves stay turned (rnd/joints.py SLEEVE_STOCK)", SLEEVE_TUBE[1]),
         ("Drive (D24 inline, W8c / D26)", "Sleeve facing jig: mini tube cutter, abrasive on a flat plate, drilled guide blocks Ø6 / Ø8", 1, FACING_JIG[0],
          "every sleeve to the measured print − 0.05 ± 0.02 (≈ 3 min each, 32 per robot)", FACING_JIG[1]),
         ("Drive (D24 inline, W8c)", "Rear plate ring, 7075-T6 3 mm, Ø96 / 70", 1, PLATE_RING[0], "all-metal rear clamp (engagement 2.5–3.5 mm, no shim picking)", PLATE_RING[1]),
         ("Drive (D24 inline, W8c)", "Rear screws **8 × M4 × 25 ISO 10642 10.9** (W16c: the 2 socket-head ones in the stop bushes became countersunk) "
          "+ 1 × DIN 471 Ø30 ring (61806 on the stub)",
          1, 5.0, "replace 8 × M4 × 18 + washers + DIN 988 shims and the 2 × Ø10 × 28 stop dowels", E),
         ("Drive (D24 inline, W8c)", "Loctite 660 (61806 outer race, 0.3 mm gap) + Loctite 638 (output dowels into Ø4.1 adapter holes)", 1, LOCTITE[0],
          "the 61806 floats ± 0.3 mm (forced load ≤ 800 N); dowels bonded seated in the motor's holes (150 MPa with load sharing)", LOCTITE[1])],
    no=[("Actuators", "SteadyWin GIM6010-8 24 V SDC104", 1, GIM6010[0], "shoulder roll on the 18T:80T belt (4.44:1)", GIM6010[1]),
        ("Drive (D26 v5)", "C45+QT integral pinion shaft, **18T** hobbed, + 6001-2RS (direct stage)", 1, PINION[0] + B6001[0], "roll stage (18T:80T)",
         PINION[1] + "; 6001: " + B6001[1]),
        ("Drive", "HTD 5M-15 closed belt 475 mm", 1, BELT[0], "roll (80T)", BELT[1]),
        ("Bearings", "61808 (40×52×7)", 2, B61808[0], "roll one-piece shaft (D19)", B61808[1]),
        ("Thermal (M7)", "Aluminium finned rear cap, Ø86 class", 1, AL_CAP[0], "roll GIM6010 cartridge", AL_CAP[1])],
    note="The CAD and checks for both drives are in the tree (`PIP_ROLL_DRIVE=inline`, roll_inline.md). Inline-specific risks (Fable M1): "
         "the output boss joint is carried by its bonded dowels (1.03× stall on an assumed cast-alloy boss, joints.md), the hard stop is metal "
         "(W16c: 7075 dogs on the adapter against a 7075 ring under the front collar, 3.91× at stall), and the 36:1 planetary adds ≈ 15 arcmin of backlash "
         "(accuracy.md: 4.3 mm at the hand without joint encoders). The SDC303 is a different driver from the Step-1 SDC104: protocol discovery "
         "runs again on it on arrival.",
    hi=lambda: line_sum(HOLD_B["yes"]) - GIM8108_36[0] + GIM8108_36_HI - line_sum(HOLD_B["no"]),
)

# Exact questions for the seller (SteadyWin reseller: aifitlab / Robotics Center / openelab), one message.
SELLER_QUESTIONS = [
    "**GIM8108-8, 24 V:** we run a 24 V bus (6S Li-ion, 25.2 V full; our regen clamp switches on at 26.0 V ± 1 % and off at 25.7 V, and the drive end "
    "stays ≤ 27.5 V at our regen limit; please confirm the drive's absolute maximum input voltage). Which driver "
    "version of the GIM8108-8 (SDC104, GDZ468, GDS68?) do you ship, and is it specified for 24 V? Please give rated torque, rated speed, "
    "no-load speed and continuous current at 24 V (listings quote 48 V). Price and lead time for 2 now (4 later).",
    "**GIM8108-8 drawing:** installation drawing with frame diameter and height, output-flange bolt pattern (hole count, PCD, thread, depth) "
    "and housing / rear mounting pattern, and the connector position.",
    "**GIM8108-36 (SDC303, 24 V):** which face is the output; the output flange material and the dowel-hole depth; reducer **backlash** "
    "(listings say 15 arcmin: measured how?); frame and connector position (we have the Ø96 × 57 drawing).",
    "**GIM8108-36 output bearing:** please give the **bearing type** (deep-groove, angular-contact pair, crossed-roller?) and its **radial, axial and "
    "tilting-moment ratings** (static and dynamic, and where along the output they are referenced). Our two load cases at the output flange "
    "(rnd/joints.py, M1 roll support, Astra review): **with our rear 61806 support, ≈ 1,413 N radial in service and 3,130 N radial at a fault** "
    "(a collision or a stall, including misalignment); **without it, a tilting moment of 36.4 N·m in service and 137 N·m at a fault**. One listing gives "
    "1000 N radial / 250 N axial: please confirm, and say which of the two cases the bearing carries. Price with the SDC303 driver and lead time.",
    "**CAN protocol manual** for the SDC104 (GIM6010-8), SDC303 (GIM8108-36) and the GIM8108-8's driver: frame formats; is the MIT / "
    "impedance mode (position, velocity, kp, kd, torque feed-forward) the default or selected in the tool, and its float ranges; how to "
    "set the node ID and the feedback ID; 1 Mbit/s; a **CAN-command timeout** (the drive disables itself when commands stop) and its "
    "unit; a torque / current-limit parameter **saved to flash**; encoder zero (is the output angle kept over a power cycle?); status and "
    "error codes; under/over-voltage behaviour. **Is it the same protocol on all three drivers?**",
    "**GIM6010-8:** confirm the unit is the 24 V **SDC104** version (not a 48 V listing), with its installation drawing and which face "
    "carries the housing mounting holes (rear or output face: our cartridge cap depends on it, Fable C3); the output flange's bolt circle, pilot "
    "diameter and thread depth; and the **output bearing** (single deep-groove row, angular-contact pair or crossed roller?) with its radial and "
    "tilting-moment ratings (our pinion is supported by it and a second bearing, Fable #2 M3).",
]


# Options: priced, never in a total (an owner decision pending; the CAD is behind a params switch)
OPTIONS = [
    ("Option (R17, D25 / Astra #3 F1)", "**Roll latch** (PIP_ROLL_GUARD=latch, default off; power_off.md option O2): spring-applied Ø10 hardened pin in a 42CrMo4 "
     "guide sleeve at roll +5°, 24 V pull solenoid (JF-0730B class) on a cable, pin-retracted hall switch + magnet, supervisor-gated FET", 1, *ROLL_LATCH[:1],
     "a safe unpowered roll state (the arm cannot be pushed into the inboard wedge); ≈ 45 g per arm; the print behind the sleeve carries ≈ 0.4 kN until a "
     "metal path to the motor face is drawn (power_off.md)", ROLL_LATCH[1]),
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
    "**W17a / W17b (2026-11-28): the Step-1 rig priced, safety board v4.** The $0 rig placeholder in 1b is replaced by W17a's lines "
    "(docs/M1_RIG.md §5, ESTIMATE): in 1a the rig stand ($75, replaces the $30 stand line and keeps its 2 bench clamps), the 6061 stock, the turned "
    "tilt mandrel + 6001-2RS, two 0.01 mm dial indicators with bases and a flat bar, the optical lever + turnbuckle + Dyneema, a 5 kg / 1 g kitchen "
    "scale and the rig fasteners; in 1b the second bar, 8 steel discs + 4 carrier plates (replacing the 1 kg / 2 kg weights, −$12), 4 rubber "
    "buffers, 1.39 m² of 3 mm polycarbonate and the magnetic safety switch. The category-0 catch ($25) moves from 1b to Step 2. W17a's net was "
    "+$302 (+$347 landed); **merged with W17c's coupon tools it is +$292 (+$336 landed):** the laser + mirror were in both the C14 tool line and "
    "the rig's optical lever (the C14 line keeps only its 0.002 mm indicator, now the third indicator on the housing rim: −$10), and the C14 "
    "dummy output is cut from the rig's 10 mm plate (ordered 300 × 210: the offcut line keeps the S-hook + bucket at $5, the plate goes $35 → $38; "
    "net $0). The weights left the 1a lever-stock line, so 1a carries their −$12. W17b: + $1.50 of v4 logic on the board line (U28 SN74LVC1G74, "
    "U29 74LVC1G17, BAT54, 47 nF C0G, two 1 % resistors); the RH-50 0.56 Ω with its freewheel Schottky is the accepted NH-50 alternate; the G-SB "
    "review is an optional service line (route A hired EE $400–1,200 ESTIMATE, route B community review $0), outside every total.",
    "**Fable review #2 (2026-11-28): Step 1 split into 1a and 1b.** 1a (buy now) is what protocol discovery on a bare, unloaded GIM at a 1 A limit "
    "needs (motors, CAN, the 10 A supply, stand, lever stock, thermistors, inserts, screws, filament) plus the coupon tools for C12–C14; 1b (the "
    "SE-1000-24, the safety board, the catch, the test kit and instruments, the pinion and the rig parts) waits for the four-item gate. "
    "The differential probe is dropped (−$175: low-side drains are read with a ×10 probe and a ground spring); the SE-1000 is set to 24.0 V; "
    "the pinion is quoted only after C14 measures the GIM output's tilt stiffness (M3); a placeholder line holds the rig parts (rig track). "
    "Seller question 1 quotes the board's clamp thresholds (26.0 / 25.7 V; it said 26.4 / 27.3) and question 6 asks for the GIM6010-8 output bearing.",
    "**W16c R17 roll stop (2026-11-28, Hold B yes):** the 2 stepped stop bushes (−$20 an arm) and their 2 socket-head M4 × 25 go (all 8 rear screws "
    "ISO 10642, + 2 rear sleeves from the tube stick); the output adapter blank 7 → 19.5 mm for the two dogs (+$8); a 7075 front stop ring with two "
    "blocks ($35 ESTIMATE), 8 × M4 × 6 ISO 10642 and 2 × M3 × 10 (+$3). Robot: the W14b collars' 8 × M4 × 25 go (rnd/bom.py). The W13 stainless wear "
    "strips were never a priced line.",
    "**W15b electrical (2026-11-28):** the Step-1 test kit gets a second VS-40CPQ060-N3 and a 0.33 Ω 50 W resistor for T12's 30 A step (the 20 A supply "
    "plus the 22 mF bank at 32.5 V): +$8 of lines, Step 1 $2,835 → $2,844 landed. The CP2100A stays (W15b accepted 800 kHz: the requirement is now "
    "≥ 500 kHz). The W15b base channel (Q9 base switch, base clamp leg, U20 OFF-delay timer, WD_B / DISC_B, the extra NC blocks) is robot-only: "
    "rnd/bom.py, not this kit (the M1 bench has no base).",
    "**D29 cost route (2026-11-28, cost_down.md):** GIM6010-8 at Robotics Center's $70 (backordered) in Step 2 and the holds, with aifitlab's "
    "$90 as the fallback; Step 1's one unit stays at the $90 fallback because Step 1 is bought now. NH-50 clamp resistors $30 → $18 ESTIMATE "
    "(NH050R5600FE02; no 0.56 Ω listing found, NH-50 single-unit listings $12–20). The arm-detach clamp follows PIP_DETACH_CLAMP. Not adopted: "
    "the stock rear-cap heatsink (thermal re-check fails, and the retail listing is dearer than the CNC cap) and the MT6701 encoders.",
    "**Waves 12–13 refresh (2026-11-28, Astra review #3):** the drive stub is now a **C45+QT integral pinion shaft** with the 20T hobbed on it (stub v5, m7: "
    "the keyed v4 stub was 0.68× / 1.19× in rotating-bending fatigue): Step 1's stub line becomes the pinion ($35 ESTIMATE, belts.md), and Step 2 no "
    "longer buys the yaw's keyed aluminium 20T or the key assortment; hold A / B pulleys + stubs become pinions. **D26 tension rings** (turned 6061-T6, "
    "e 0.30 mm; m6) at the yaw (common) and the elbow (hold A yes / no). **C1 faces (m9):** both clamp rings C45+QT (were 6061 / S235), the yaw flange "
    "7075-T6 (was 6061), yaw screws M3 × 40. **Safety board netlist v3 (critical 3):** a 100 V coil-driver MOSFET, the 50 µs minimum-on one-shot, the "
    "AUX / K1 discrepancy monitor, the 12 V UVLO and the arm-branch current trip are a new Step-1 line. **Step-1 gate test kit** (motorless tests T0–T16): "
    "dummy drive bank, load resistors, regen diode, energy bank, and the instruments (scope, differential probe, two current clamps, a 20 A CC supply, "
    "DMM). **Option:** the R17 roll latch ($14 per arm ESTIMATE, PIP_ROLL_GUARD=latch, default off), never in a total.",
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
    "SE-1000-24 (1b): the 24 V model (not -12 / -48); check the overload mode on its datasheet (a latched shutdown, not constant-current limiting) and set "
    "**24.0 V** (Fable #2 minor: at 25.2 V the margin to the clamp's 25.44 V lowest turn-off is 0.24 V against ± 1 % and 150 mV ripple).",
    "GIM6010-8 on arrival (1a, C9 + C14): read the output PCD, pilot and housing pattern with the C9 disc; measure the output's tilt under a known moment "
    "and note the bearing layout (C14) **before** the integral pinion is quoted (Fable #2 M3).",
    "Drive stub (D26 v5): set belt_stage.STUB / MOTOR['out_pcd'] from C9 / C14 (output bolt circle, pilot, thread depth, bearing position and "
    "tilt) before quoting the pinions from the STEP; no keyed pulley or keyway any more (the 20T is hobbed on the shaft).",
    "Crush sleeves (W8c, D26): measure each printed stack; pitch / yaw sleeves turned 42CrMo4+QT (R_p0.2 ≥ 750 MPa on the certificate) to print − 0.05 mm; clevis / rear sleeves cut from the E235+C tube sticks and faced in the jig to print − 0.05 ± 0.02 (micrometer).",
    "Vishay NH-50 for the three clamp resistors: the part number starts NH050 (0.56 Ω 1 %: NH050R5600FE02; confirm the code with the "
    "distributor, D29 #7). **Accepted alternate (W17b):** the RH-50 0.56 Ω (RH050R5600FE05 class), only with the freewheel Schottky fitted across "
    "each resistor. The pre-charge and dummy-load resistors stay RH-50.",
    "GIM6010-8 at $70 (D29 #2): Robotics Center is backordered: confirm the SDC104 24 V version in writing (seller question 6) and the ship date "
    "before paying; fall back to the $90 aifitlab listing if it would hold Step 2 up.",
    "Heat-set inserts: the short M4 (5.7 mm) the flange calls for; the long 8.1 mm ones don't fit the 6 mm flange.",
    "Integral pinion shafts (v5): quote from the STEP after C14 (PCD and the hinged / overhung choice); a single piece may cost $100+ (Fable #2 M3, S); check the hobbed 20T against a stock HTD 5M belt (pitch diameter "
    "31.83 mm, root Ø26.6), the Ø12 k5 seat and the C45+QT certificate (R_e ≥ 490 MPa).",
    "C1 clamp rings: C45+QT on the certificate (R_e ≥ 490 MPa), not plain C45 or S235; the yaw flange 7075-T6 (T651 plate), not 6061.",
    "Test instruments (1b): the 100 A clamp ≥ 500 kHz (the CP2100A's 800 kHz is enough: W15b, safety_board_tests.md *Current-clamp bandwidth*); no "
    "differential probe (Fable #2 minor): TP3 / TP7 with a ×10 probe and a ground spring to board ground.",
]


def totals():
    c = 1 + CONTINGENCY
    s1a, s1bp, s1t = line_sum(step1a()), line_sum(step1b()), line_sum(step1_test())
    s1a_tools = line_sum([r for r in step1a() if "tools" in r[0]])
    s1i = line_sum([r for r in step1_test() if "instruments" in r[0]])
    s1b = s1bp + s1t
    s1p = s1a + s1bp
    s1, s2 = s1a + s1b, line_sum(step2_common())
    a_yes, a_no = line_sum(HOLD_A["yes"]), line_sum(HOLD_A["no"])
    b_yes, b_no = line_sum(HOLD_B["yes"]), line_sum(HOLD_B["no"])
    plan2 = s2 + a_no + b_yes                     # today's plan: GIM6010 at pitch + elbow, inline roll
    rig = lambda rows: line_sum([r for r in rows if r[0].startswith("Step-1 rig")])
    return dict(s1a=s1a, s1a_c=s1a * c, s1a_tools=s1a_tools, s1a_tools_c=s1a_tools * c,
                rig_1a=rig(step1a()), rig_1b=rig(step1b()),
                gsb_lo=sum(r[2] for r in SERVICES), gsb_hi=sum(r[3] for r in SERVICES),
                s1b=s1b, s1b_c=s1b * c, s1b_parts=s1bp, s1b_parts_c=s1bp * c, s1b_instr=s1i, s1b_instr_c=s1i * c,
                s1=s1, s1_c=s1 * c, s1_parts=s1p, s1_parts_c=s1p * c, s1_test=s1t, s1_test_c=s1t * c,
                s1_instr=s1i + s1a_tools, s1_instr_c=(s1i + s1a_tools) * c,
                opt=line_sum(OPTIONS), opt_c=line_sum(OPTIONS) * c, s2=s2, a_yes=a_yes, a_no=a_no, b_yes=b_yes, b_no=b_no,
                s2_plan=plan2, s2_plan_c=plan2 * c, s2_d20_c=(s2 + a_yes + b_yes) * c, s2_belt_c=(s2 + a_no + b_no) * c,
                d_a=(a_yes - a_no) * c, d_b=(b_yes - b_no) * c, d_a_hi=HOLD_A["hi"]() * c, d_b_hi=HOLD_B["hi"]() * c,
                plan_c=(s1 + plan2) * c, d20_c=(s1 + s2 + a_yes + b_yes) * c, belt_c=(s1 + s2 + a_no + b_no) * c,
                # old keys (m1_readiness / bom callers)
                now=s1a, now_c=s1a * c, default=(s1 + s2 + a_no + b_no) * c, both=(s1 + s2 + a_yes + b_yes) * c)


def _table(L, rows):
    L += ["| Group | Item | Qty | Unit $ | Line $ | Why | Price source |", "|---|---|---|---|---|---|---|"]
    L += [f"| {g} | {n} | {q} | {u:,.2f} | {q * u:,.0f} | {w} | {s} |" for g, n, q, u, w, s in rows]


def _sub(L, x, label):
    c = CONTINGENCY
    L += [f"| | **{label} subtotal** | | | **{x:,.0f}** | | |", f"| | Contingency {c:.0%} (A16) | | | {x * c:,.0f} | | |",
          f"| | **{label} total** | | | **{x * (1 + c):,.0f}** | | |", ""]


STEP0 = [
    "**Read the public GIM6010-8 manual** (SteadyWin rev 1: the first source under GIM6010-8 in `rnd/specs/actuators.yaml`, "
    "https://sdalal1.github.io/assets/pdf/SteadyWin_GIM6010-8_Manual_rev1.pdf). Look for: the CAN frame formats and whether MIT / impedance mode "
    "is the default, the CAN-command timeout, a torque / current limit saved to flash, the node-ID setting, the output bolt circle and the output "
    "bearing. Write what it says into docs/sw/gim_protocol_notes.md; anything it settles comes off the discovery list.",
    "**Send the six seller questions below as one message** (SteadyWin reseller), and the DM-J4310 bearing question to Damiao. Answers to 5 and 6 "
    "can change 1a (a different driver version, a different output pattern); the rest gate 1b and Step 2.",
]


def main():
    S1A, S1B, S1T, S2 = step1a(), step1b(), step1_test(), step2_common()
    t = totals()
    c = CONTINGENCY
    est = lambda rows: sum(1 for *_, s in rows if s.startswith("ESTIMATE"))
    S1 = S1A + S1B + S1T
    L = ["# Milestone M1: one-arm bench kit, Step 1a / 1b / Step 2 (generated by `python -m rnd.m1_kit`)", "",
         f"**Step 1a, buy now (protocol discovery on a bare GIM at 1 A, no safety board): ${t['s1a']:,.0f} + {c:.0%} = ${t['s1a_c']:,.0f}** "
         f"(measuring tools ${t['s1a_tools_c']:,.0f} of it: skip what the owner has; the rig's stand, stock, tilt mandrel, indicators and scale "
         f"${t['rig_1a']:,.0f} of lines). "
         f"**Step 1b, after the gate (supply, safety board, test kit, rig, pinion): ${t['s1b']:,.0f} + {c:.0%} = ${t['s1b_c']:,.0f}** "
         f"(test instruments ${t['s1b_instr_c']:,.0f} of it: skip any the owner owns → ${t['s1b_c'] - t['s1b_instr_c']:,.0f}; the rig's loaded parts "
         f"${t['rig_1b']:,.0f} of lines, W17a). **Optional service, outside every total:** the G-SB board review, route A a hired EE "
         f"${t['gsb_lo']:,.0f}–{t['gsb_hi']:,.0f} (ESTIMATE), route B community review $0. "
         f"**Step 2 (the rest of the arm, today's plan): ${t['s2_plan']:,.0f} + {c:.0%} = ${t['s2_plan_c']:,.0f}.** "
         f"Whole kit ${t['plan_c']:,.0f} with contingency (Fable #2 quoted ≈ $720 / ≈ $2,115 for 1a / 1b on the W16b list). "
         f"Option, not in any total: the R17 roll latch {sd(t['opt'])} per arm ({sd(t['opt_c'])} with contingency).", "",
         "| Scenario (with contingency) | Step 1a | Step 1b | Step 2 | Whole kit |", "|---|---|---|---|---|",
         f"| **Today's plan**: GIM6010 at pitch + elbow, approved inline roll | ${t['s1a_c']:,.0f} | ${t['s1b_c']:,.0f} | ${t['s2_plan_c']:,.0f} | **${t['plan_c']:,.0f}** |",
         f"| Hold A yes (D20, with its drive parts) + inline roll | ${t['s1a_c']:,.0f} | ${t['s1b_c']:,.0f} | ${t['s2_d20_c']:,.0f} | ${t['d20_c']:,.0f} |",
         f"| Fallback: belt roll (hold B no), GIM6010 at pitch + elbow | ${t['s1a_c']:,.0f} | ${t['s1b_c']:,.0f} | ${t['s2_belt_c']:,.0f} | ${t['belt_c']:,.0f} |", "",
         f"Hold A (D20) costs **{sd(t['d_a'])}** against the GIM6010 lines (up to {sd(t['d_a_hi'])} at the dearest motor listing); the inline roll "
         f"{sd(t['d_b'])} against the belt roll (up to {sd(t['d_b_hi'])}). Step 1: {len(S1) - est(S1)} of {len(S1)} lines priced from retailer snapshots, "
         f"{est(S1)} ESTIMATE. Everything except the supplies, the rig (stand, levers, discs, stops, guard), the catch, the "
         "measuring and coupon tools and the test kit is reused in the arm and the robot.", "",
         "**Why 1a / 1b (Fable review #2, 2026-11-28, verdict):** discovery runs on one unloaded motor at a 1 A limit (`sw/gim_discover.py`) and "
         "needs none of the supply, board or instruments. If the SDC104 has no impedance mode, no CAN timeout or no flash limit, the repo's own "
         "answer is \"stop and redesign the safety argument\", and the board's timing budget and the bench plan are part of that argument; the board "
         "was a netlist with an open logic race (M2; fixed on paper in netlist v4, W17b; not yet drawn or reviewed), and the lever rig had no stops, "
         "guard or software description (C1; designed in W17a: docs/M1_RIG.md). So 1b waits for the facts that decide it. Why two steps at all (Fable #1): no GIM can be commanded until its protocol is known, and the arm mixes three "
         "SteadyWin drivers (docs/sw/gim_protocol_notes.md). **No extra GIM6010s in Step 1.**", "",
         "## Step 0: free, before any money", ""]
    L += [f"{i}. {x}" for i, x in enumerate(STEP0, 1)]
    L += ["", "## Step 1a: buy now (bare-motor protocol discovery and the coupons)", "",
          "**What 1a proves:** on the bare, unloaded GIM6010-8 at a 1 A limit, alone on can0, the drive enables, takes a zero-gain then a low-gain "
          "impedance frame, disables itself on the **CAN timeout**, and keeps a **torque limit in flash** over a power cycle (`python -m sw.gim_discover`, "
          "then the vendor tool / manual registers); the DM-J4310 answers on the same bus; the CAN HAT against the two adapters; coupons C9 (bolt "
          "circles), C12 (insert pull-out), C13 (4-ear flange) and C14 (drive stub: the output's tilt stiffness, bearing layout, PCD and pilot, "
          "and the 8 × M3 joint to 1.5 × the peak on a dummy output; Fable #2 M3) ([COUPONS.md](COUPONS.md)). The procedure is docs/M1_RIG.md §3 "
          "(`python -m sw.rig_1a`: limits in flash, impedance, the drive's CAN timeout, stall against the luggage scale, encoder zero; 1a-11 the GIM "
          "output's tilt stiffness on the tilt mandrel). **Step 1a exit criterion:** `sw.rig_1a` all phases pass on the GIM and the DM; GIM tilt "
          "stiffness measured and decided (k_t ≤ 8 kN·m/rad keeps the D26 6001 support, ≥ 75 overhung, in between the lead re-runs the split). "
          "**No lever weight on the motor in 1a.**", ""]
    _table(L, S1A)
    L += [""]
    _sub(L, t["s1a"], "Step 1a")
    L += ["## Gate: buy Step 1b only when all four are met", ""]
    L += [f"{i}. {x}" for i, x in enumerate(STEP1B_GATE, 1)]
    L += ["", "## Step 1b: the loaded rig and the safety chain (after the gate)", "",
          "**What 1b proves:** the safety board (pre-charge, one-shot discharge, clamp, contactor, watchdog at 25 ms, e-stop → no torque ≤ 50 ms) "
          "with a real drive; torque constant (from the weighed lever) and friction on the W17a lever rig (`--rig step1`; rubber stops at −15° / +100°; "
          "guard interlocked); the collision trip on one joint; "
          "the motor case temperature rise at rated torque; the C14 joint at the drive's peak on the real output; the soak.", ""]
    _table(L, S1B)
    L += [f"| | **Step 1b parts subtotal** | | | **{t['s1b_parts']:,.0f}** | | |", "",
          "### Step-1b gate equipment: motorless test kit (docs/rnd/safety_board_tests.md *Equipment*, tests T0–T16)", "",
          "The board must pass T0–T16 with no motor connected before any drive gets bus power from it. The kit lines are consumed by the tests; the "
          "instrument lines are tools (the robot BOM excludes tools): skip any the owner already owns or can borrow.", ""]
    _table(L, S1T)
    L += [f"| | **Test kit subtotal** (instruments {t['s1b_instr']:,.0f}) | | | **{t['s1_test']:,.0f}** | | |", ""]
    _sub(L, t["s1b"], "Step 1b")
    L += ["### Step 1b service (optional, **outside every parts total**)", "",
          "| Group | Service | Low $ | High $ | Why | Price source |", "|---|---|---|---|---|---|"]
    L += [f"| {g} | {n} | {lo:,.0f} | {hi:,.0f} | {w} | {s} |" for g, n, lo, hi, w, s in SERVICES]
    L += ["", f"Route A adds ${t['gsb_lo']:,.0f}–{t['gsb_hi']:,.0f} (ESTIMATE, no contingency) to the cash the owner spends before 1b; route B costs $0 "
          "and 1–2 more weeks. Neither is a part, so neither is in Step 1b, the kit or the robot total.", ""]
    L += [f"**Step 1 (1a + 1b): ${t['s1']:,.0f} + {c:.0%} = ${t['s1_c']:,.0f}.**", "",
          '**Step-1b exit criterion (software track: determinism, the Asimov sim2real lessons): soak test.** `python -m sw.soak --rig step1 --load soak '
          '--joints yaw,wpitch --hours 2` on the W17a rig (docs/M1_RIG.md §4, 1b-4). A sysid fit at the start (bring-up step `sysid`, sw/sysid.py: '
          "Coulomb + viscous friction, effective torque constant against the weighed lever, backlash, command → response latency → "
          "sw/config/rig_step1/actuators_<serial>.yaml), then 2 h of 0.25 Hz ±15° cyclic motion: "
          "**soak discs 4 / 2 at 0.27 m (GIM / DM; 1.04 / 0.60 kg on the bolt), RMS 45–65 % of rated** (sized 55 / 57 %; Fable #2 C1: the old 2 kg "
          "at 0.3 m was 118 % of the bare motor's rated); then the sysid fit again with the drives hot. **Pass:** Coulomb drift ≤ 20 %, viscous ≤ 25 %, torque constant ≤ 5 %, backlash growth ≤ 0.1°, latency mean ≤ +0.5 ms and "
          "p95 ≤ +2 ms; no supervisor stop, no CAN bus-off, no new Pi throttle / under-voltage flag (`vcgencmd get_throttled`), zero implausible CAN "
          "frames, loop jitter p99 < 1 ms, CPU < 80 °C (docs/sw/sim2real_bench.md).", "",
          "**What Step 2 proves:** the whole arm: belts and the D26 direct stages, the bolted cartridges, the balancer (A9), the inline roll, hand "
          "sag (R2b) and, if the node firmware exists by then (Fable M3), hand accuracy; the fin-ray grasp; CAN timing (SW1).", "",
          "## Step 2: the rest of the arm (after Step 1b's exit criteria)", "", "### Common to every Step-2 scenario", ""]
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
          "### Option (not in any total)", "", "Priced so the owner's decision costs nothing to make; bought with Step 2 only if the owner picks it.", ""]
    _table(L, OPTIONS)
    L += ["",
          "## Questions for the seller (Step 0: send as one message now; 5 and 6 can change 1a, the rest gate 1b and Step 2)", ""]
    L += [f"{i}. {q}" for i, q in enumerate(SELLER_QUESTIONS, 1)]
    L += ["", "Ask Damiao (or the DM-J4310 reseller) separately: the V1.1 output bearing's allowable radial load and moment.", "",
          "## What changed since the loop-47 list", ""] + [f"- {x}" for x in CHANGES] + [
          "", "## Check before paying / on arrival", ""] + [f"- {x}" for x in CHECKS] + [
          "", "**Order of work:** Step 0 (manual, seller questions) → 1a: discovery on the bare GIM6010-8 at 1 A, C9 / C14 on the motor, C12 / C13 / the C14 "
          "joint on the bench → the 1b gate (discovery result, the board drawn + reviewed with the race fixed, the rig designed, C14 read) → 1b: build "
          "and test the safety board (T0–T16, no motor) → bring-up on the rig (`--joints yaw,wpitch`) → soak → Step 2. Readiness and the go / no-go per "
          "step: [M1_READINESS.md](M1_READINESS.md). Buying is the owner's decision."]
    (DOCS / "M1_BENCH_ARM.md").write_text("\n".join(L) + "\n")
    print("\n".join(L[2:4]))
    return t


if __name__ == "__main__":
    main()
