"""Milestone M1: one-arm bench kit (shoulder pitch module + roll/yaw + arm + hand on a desk clamp), as three buy lists.
python -m rnd.m1_kit

Wave-1 refresh (2026-11-27, owner about to buy): the list follows the design after wave 1 (D24 bolted cartridges with an
Oldham drive, D19 one-piece shaft-pulleys in capped seats, A5 v2 electrics, the sw/bringup.py day-1 tool) and is split
by what is still open:
- **Buy now**: decided, no open question.
- **Hold A (D20)**: GIM8108-8 at shoulder pitch + elbow, waiting on the seller's 24 V confirmation.
- **Hold B (D24, owner-approved 2026-11-27)**: inline GIM8108-36 at shoulder roll; order once the seller's drawing confirms the fit.
Each hold lists what it buys if the answer is yes and the default lines it replaces (today's params) if no.
Every price is a retailer snapshot (source named) or ESTIMATE.
"""
from __future__ import annotations

from pathlib import Path

DOCS = Path(__file__).resolve().parent.parent / "docs"
CONTINGENCY = 0.15            # A16: price drift, shipping and tax (review finding 18)
E = "ESTIMATE"

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

# (group, item, qty, unit $, why / where, source)
def items():
    """Buy now: decided, no open question (one arm + bench)."""
    j, s = (150.0, "Robotics Center $150 in stock (2026-11-27); AIFIT $125 backordered"), (19.5, "loop 14 retailer snapshot")
    return [
        ("Actuators", "SteadyWin GIM6010-8 **24 V, SDC104 driver version**", 1, *GIM6010[:1], "upper-arm yaw (20T:40T belt); unchanged by either hold", GIM6010[1]),
        ("Actuators", "Damiao DM-J4310-2EC **V1.1**", 1, *j[:1], "wrist pitch, bolted output + can flange (D24)", j[1]),
        ("Actuators", "Feetech STS3215-C018 **12 V** (30 kg·cm)", 2, *s[:1], "wrist roll (D16 spindle), gripper", s[1]),
        ("Drive (D13 v3)", "Steel rod Ø12 h6, 300 mm (cut to 4 × 50 mm stage shafts)", 1, 9.0, "pitch, roll, yaw, elbow stages (v3 shaft 49.5 mm); a spare cut", E),
        ("Drive (D13 v3)", "Aluminium HTD 5M pulley, 20T, 15 mm belt, **12 mm bore**, 2 set screws", 3, *AL_PULLEY[:1], "pitch, yaw, elbow small pulleys", AL_PULLEY[1]),
        ("Drive (D24)", "Oldham coupling, D40, **12 mm clamp hub**, acetal (POM) disc (the second hub is a spare)", 3, *OLDHAM[:1],
         "pitch, yaw, elbow: shaft-side hub + disc; the motor-side tongue adapter is printed", OLDHAM[1]),
        ("Drive (D24)", "Acetal (POM) rod Ø40 × 100 mm", 1, 12.0, "spare Oldham discs (10 mm), cut to fit if a bought disc's slot doesn't match the printed tongue", E),
        ("Drive", "HTD 5M-15 closed belts: 355 (pitch, 20T:60T), 315 (yaw, 20T:40T) mm", 2, *BELT[:1], "nearest stock to 354 / 313 (rnd/belts.py)", BELT[1]),
        ("Bearings", "61808 (40×52×7)", 6, *B61808[:1], "pitch, yaw, elbow one-piece shafts (D19), 2 each", B61808[1]),
        ("Bearings", "6708 (40×50×6) thin", 2, 2.5, "elbow belt cassette", E),
        ("Bearings", "6801 (12×21×5)", 6, *B6801[:1], "D13 stage shafts, 2 per stage (buy a 10-pack)", B6801[1]),
        ("Bearings", "61804 (20×32×7)", 2, 0.92, "D16 wrist-roll spindle", "AliExpress MOCHU snapshot; Zoro $4.69"),
        ("Bearings", "U604ZZ grooved pulley bearing", 2, 1.5, "A9 balancer: pulley A + tackle pulley", E),
        ("Fasteners (D24/D19)", "M4 brass heat-set inserts, **short** (Ø5.6 hole × 5.7 mm), 100-pack", 1, 15.93,
         "6 per bolted cartridge flange + D19 seat caps (2 each), yaw flange, outer wall: ≈ 50 per arm", "Newark $15.93 / 100 (M4 × 8.1 Heat-Sert class; short variant ESTIMATE)"),
        ("Fasteners (D24/D19)", "M3 brass heat-set inserts (Ø4.0–4.1 hole), 100-pack", 1, 12.0, "cartridge rear caps (3 radial), forearm wrist rim (6), palm, bone cap, ferrule (6): ≈ 45 per arm", E),
        ("Fasteners (D24/D19)", "Stainless screw kit: 30 × M4 × 12 countersunk (ISO 10642), M4 / M3 SHCS assortment, 6 × M3 × 60, 8 × M5 × 30 + nylocs",
         1, 25.0, "cartridge flanges (6 × M4 csk each, from the pulley side), seat caps, palm, forearm clamp (2 × M5), cassette posts", E),
        ("Fasteners", "Hardened dowels Ø6 × 10 (yaw hard-stop pin) + Ø6 × 12 (3 × KX-V ferrule dowels)", 1, 6.0, "yaw stops ±155°, arm detach ferrule", E),
        ("Structure", "Carbon tubes Ø40×1 + Ø30×1 (one arm)", 1, 40.0, "upper-arm neck (bond ≈ 55 mm), forearm link (30 mm into each socket)", E),
        ("Structure", '3" quick-release squeeze tri-clamp, 304 SS', 1, 25.0, "KX-V ferrule at the shoulder (arm detach)", "hosewarehouse (Dixon 13MHHM-Q300)"),
        ("Balancer (A9)", "ISO 10243 green die spring Ø25 / Ø12.5 × L0 115, 18.7 N/mm (Berger Tools V25-115 or Accu TJL25-115) + 2 mm UHMWPE cord", 1, 14.0,
         "2:1 tackle balancer, pin B b 20 mm, a 40–60 mm (balancer.md)", E),
        ("Sensors", "4 × AS5048A joint encoders + 2 RP2040 CAN nodes (shoulder + elbow) + 2 tactile pads + palm ToF", 1, 96.0,
         "joint-side encoders (accuracy 0.9 mm); CAN map nodes 0x30 / 0x31", E),
        ("Sensors (D14)", "Elbow node power + bus: 24→12 V **6 A** buck (two stalled STS3215 draw 5.4 A) + half-duplex TTL transceiver + 2 × MCP2562", 1, 14.0,
         "D14: 12 V and TTL made in the arm (sw/config/can_map.yaml)", E),
        ("Sensors", "Global-shutter wrist camera (OV9281 MIPI-CSI) + CSI→USB UVC bridge + slim high-flex USB cable", 1, 70.0, "D12 wrist camera", E),
        ("Harness", "Clock-spring ribbon stock: 20 AWG silicone pair, 26 AWG twisted pairs, Kapton/PET carrier tape; JST-GH/VH kit", 1, 18.0,
         "joint crossings (rnd/harness.py; 11.3 mm cassettes, yaw ribbon 0.75 m)", E),
        ("Electronics (A14)", "USB-CAN adapter, candleLight / gs_usb (FYSETC UCAN, STM32F072), onboard 120 Ω (jumper)", 2, 19.0,
         "can_map.yaml buses B1 (pitch, roll, yaw, shoulder node) + B2 (elbow, wrist pitch, elbow node) = SocketCAN can0 / can1", "MatterHackers $19 (2026-11)"),
        ("Electronics", "USB TTL bus-servo adapter (STS3215 ID setup only)", 1, 8.0, "servo IDs 1 / 2 before they go on the elbow node's bus", E),
        ("Power (A5)", "Adjustable 0–30 V / 10 A CC/CV bench supply + 4,700 µF 35 V bus capacitor + 15 A inline fuse", 1, 76.0,
         "current-limited first power-up (1 A), then 24 V; the fuse protects the wiring", "Newegg generic 30 V 10 A CC/CV $65.71 (2026-11-27); cap + fuse ESTIMATE"),
        ("Power (A5 v2)", "Regen clamp / bus discharge: 2 × Vishay RH-50 0.56 Ω + 2 × IRFB7530 + comparator/reference (26.5 V), gate driver, film caps, "
         "85 °C thermal switch, heatsink plate; ideal-diode stage to the PSU", 1, 58.0,
         "**mandatory on a bench supply**: 136 J / 1.0 kW for one D20 arm (robot 320 J / 2 kW; electrical.md)", "RH-50 ≈ $15 (RS 5-pack); rest ESTIMATE"),
        ("Power (A5 v2)", "E-stop: 2 two-channel latching mushrooms + TE ALEV100 DC contactor (24 V coil, aux contact) + K_PRE relay + 3.3 Ω RH-50 pre-charge "
         "+ SN74LVC1G123 watchdog on its own LDO", 1, 150.0, "category-0 stop; SW2 budget 25 ms (bench 24.6 ms, electrical.md)",
         "LEV100A5ANG $92 (Master Electronics); rest ESTIMATE"),
        ("Bring-up IO (S1)", "sw/bringup.py `--estop gpiod:gpiochip0:17:27`: reset/ARM pushbutton, coil MOSFET + suppression diode/TVS, PC817 opto "
         "(aux contact → GPIO27), heartbeat lead GPIO17 → watchdog, 3 m 4-core cable for the remote mushroom, perfboard + terminal blocks", 1, 20.0,
         "step 3 (e-stop, watchdog, reset refused while pressed); operator outside the fall envelope", E),
        ("Commissioning (A5 v2)", "Category-0 catch: adjustable padded cradle (Al bar + EVA foam) + Dyneema tether ≥ 600 N + payload tray", 1, 25.0,
         "drop energy ≤ 3 J with ≤ 50 mm gap (electrical.md)", E),
        ("Consumables", "Epoxy (CF bonds), grease, thread locker, PTFE washers", 1, 15.0, "", E),
        ("Filament (A6)", "PETG 3 kg + ASA 1 kg (bolted cans, rear caps, Oldham tongue adapters) + TPU 95A 0.25 kg", 1, 90.0,
         "A6: cans in ASA (can wall up to 63 °C); one arm ≈ 3 kg (print_plan.md)", E),
    ]


# Hold A (D20): yes = the seller confirms 24 V operation (speed, torque) of the GIM8108-8's driver version.
HOLD_A = dict(
    title="Hold A: GIM8108-8 at shoulder pitch + elbow (D20)",
    waits="the seller's confirmation that the GIM8108-8 driver version runs at 24 V (rated torque and speed at 24 V), plus its drawing",
    yes=[("Actuators", "SteadyWin GIM8108-8, **24 V-capable driver version** (confirm which: SDC104 / GDZ468 / GDS68)", 2, GIM8108_8[0],
          "shoulder pitch (3:1) + elbow (3:1, 20T:60T): 1.8× rated on the 20 lb hug (R2c)", GIM8108_8[1]),
         ("Drive", "HTD 5M-15 closed belt for the elbow at 20T:60T (length from the D20 CAD, not drawn yet)", 1, BELT[0], "elbow", BELT[1])],
    no=[("Actuators", "SteadyWin GIM6010-8 24 V SDC104", 2, GIM6010[0], "pitch + elbow as today (bench-valid at 1 kg; 0.6–0.8× on the 20 lb hug)", GIM6010[1]),
        ("Drive", "HTD 5M-15 closed belt 295 mm (elbow 20T:40T)", 1, BELT[0], "elbow", BELT[1])],
    note="No other bought part changes: the Ø96 can, the 60T elbow pulley and the larger elbow housing are printed (CAD after the "
         "answer; params still carry the GIM6010). The 24 V question is the gate: most GIM8108-8 listings are 48 V nominal and one "
         "driver option (GDZ468) is rated 12–40 V, so 24 V is inside its range but the speed at 24 V is unknown.",
    hi=lambda: 2 * (GIM8108_8_HI - GIM6010[0]),
)

# Hold B (D24 option): yes = owner OK + the drawing fits the inline roll CAD (Ø100 × 60 envelope, 6 × M4 patterns).
HOLD_B = dict(
    title="Hold B: inline GIM8108-36 at shoulder roll (D24, **approved by the owner 2026-11-27**)",
    waits="only the seller's dimension drawing (frame Ø, flange patterns, output-bearing radial rating ≥ 280 N); the owner approved it 2026-11-27. The *no* list is the fallback if the drawing doesn't fit",
    yes=[("Actuators", "SteadyWin GIM8108-36, **SDC303 (24 V)** driver", 1, GIM8108_36[0], "shoulder roll, inline 36:1 (dome 163 × 219 → 117 × 100 mm, roll 27 → 48 rpm)", GIM8108_36[1]),
         ("Bearings", "61806 (30×42×7)", 1, 3.0, "rear roll support on the roll link's stub (68 mm from the motor's output bearing)", E),
         ("Drive (D24 inline)", "Output adapter, 7075-T6 Ø76 × 7 (CNC: Ø30 pilot, 3 × Ø4 H7 reamed, 3 × M4 csk, 8 × M5 tapped on PCD 64)", 1, 30.0,
          "motor Ø30 boss → printed clevis arm (a printed adapter fails: 3 × M4 into PETG ≈ 85 N·m < 103 N·m stall)",
          "ESTIMATE: JLCCNC / PCBWay-class instant quote ≈ $20–25 each at qty 2 + shipping share"),
         ("Drive (D24 inline)", "Ø80/Ø52 × 2 steel clamp ring + 8 × M5×16 DIN 7984 10.9, 8 × M4×18 ISO 4762, 3 × M4×10 ISO 10642, 3 × Ø4 m6 dowels", 1, 11.0,
          "adapter ↔ clevis arm clamp, motor rear face into the roll-link floor, boss ↔ adapter", E)],
    no=[("Actuators", "SteadyWin GIM6010-8 24 V SDC104", 1, GIM6010[0], "shoulder roll on the 18T:80T belt (4.44:1)", GIM6010[1]),
        ("Drive (D13 v3)", "Aluminium HTD 5M pulley, 18T, 15 mm, 12 mm bore", 1, AL_PULLEY[0], "roll small pulley", AL_PULLEY[1]),
        ("Drive (D24)", "Oldham coupling D40, 12 mm clamp hub, POM disc", 1, OLDHAM[0], "roll stage", OLDHAM[1]),
        ("Drive", "HTD 5M-15 closed belt 475 mm", 1, BELT[0], "roll (80T)", BELT[1]),
        ("Bearings", "61808 (40×52×7)", 2, B61808[0], "roll one-piece shaft (D19)", B61808[1]),
        ("Bearings", "6801 (12×21×5)", 2, B6801[0], "roll D13 stage", B6801[1])],
    note="The CAD and checks for both drives are in the tree (`PIP_ROLL_DRIVE=inline`, roll_inline.md): R14/R15/R16/basket pass "
         "inline; R16 across-body 62 % inline vs 52 % belt (target 54 %). The inline CAD assumes Ø100 × 60 and 6 × M4 patterns "
         "(ESTIMATE); one listing gives Ø96 × 57 with the driver and 1000 N radial / 250 N axial, which would fit.",
    hi=lambda: line_sum(HOLD_B["yes"]) - GIM8108_36[0] + GIM8108_36_HI - line_sum(HOLD_B["no"]),
)

# Exact questions for the seller (SteadyWin reseller: aifitlab / Robotics Center / openelab), one message.
SELLER_QUESTIONS = [
    "**GIM8108-8, 24 V:** we run a 24 V bus (6S Li-ion, 25.2 V full; our regen clamp holds it ≤ 26.5 V, absolute max 28 V). Which driver "
    "version of the GIM8108-8 (SDC104, GDZ468, GDS68?) do you ship, and is it specified for 24 V? Please give rated torque, rated speed, "
    "no-load speed and continuous current at 24 V (listings quote 48 V). Price and lead time for 2 now (4 later).",
    "**GIM8108-8 drawing:** installation drawing with frame diameter and height, output-flange bolt pattern (hole count, PCD, thread, depth) "
    "and housing / rear mounting pattern, and the connector position.",
    "**GIM8108-36 (SDC303, 24 V) drawing:** frame diameter and overall height with the driver (one listing says Ø96 × 57 mm), output-flange "
    "pattern (hole count, PCD, thread, depth, centring spigot Ø), rear / housing mounting pattern, connector position.",
    "**GIM8108-36 output bearing:** bearing type and allowable radial load at the output flange: we need **≈ 280 N radial** continuous, "
    "with a second support bearing 68 mm behind the flange; also the allowable axial load and tilting moment (one listing gives 1000 N "
    "radial / 250 N axial: please confirm and say where it is measured). Price with the SDC303 driver and lead time.",
    "**CAN protocol manual** for the SDC104 (GIM6010-8), SDC303 (GIM8108-36) and the GIM8108-8's driver: frame formats; an MIT / "
    "impedance mode (position, velocity, kp, kd, torque feed-forward) and its float ranges; how to set the node ID and the feedback ID; "
    "1 Mbit/s support; the CAN-timeout (heartbeat) setting; a torque / current-limit parameter saved to flash; encoder zero; status and "
    "error codes; under/over-voltage behaviour. Is it the same protocol on all three drivers?",
    "**GIM6010-8:** confirm the unit is the 24 V **SDC104** version (not a 48 V listing), with its installation drawing.",
]


def sd(x):
    """Signed dollars: +$57 / −$12."""
    return f"{'+' if x >= 0 else '−'}${abs(x):,.0f}"


def line_sum(L):
    return sum(q * u for _, _, q, u, _, _ in L)


CHANGES = [
    "**D24 bolted cartridges:** the KX socket hardware is gone from the arm (30 dowels, 15 G25 balls, 30 coil springs, retainers: −$34) and so are "
    "the cartridge torque pins (−$6; the flange screws take the torque). In: Oldham couplings (one per GIM stage) + acetal rod for spare "
    "discs, M4 / M3 heat-set inserts, the countersunk flange screws. The 4 rigid flange couplings of the D13 v2 stage go (−$20): the Oldham "
    "clamp hub sits on the shaft instead.",
    "**D19 one-piece shafts:** pitch, roll, yaw and elbow big pulleys are printed on their Ø40 tubes; no split pulleys, keys or double-D "
    "parts to buy. The seat caps are printed and bolted (2 × M4 / M3 into inserts).",
    "**A5 v2** (verified in this list): energy-rated regen clamp, DC contactor with aux contact, switched pre-charge, watchdog, bench catch.",
    "**Bring-up tool (sw/bringup.py):** 2 × $19 candleLight adapters (can0 / can1 per can_map.yaml, onboard 120 Ω for the adapter end) in place "
    "of 2 × $35; the GPIO / reset-button / opto parts for `--estop gpiod`; an adjustable current-limited supply for the 1 A first power-up.",
    "**Split into three lists:** the pitch, elbow and roll motors and the parts that depend on them move into the two holds.",
]

# On arrival / before paying for the buy-now list
CHECKS = [
    "GIM6010-8: the **24 V SDC104** version; DM-J4310-2EC: **V1.1** (ask for the output bearing's radial / moment rating: the wrist "
    "pitch sees ≈ 0.9 N·m bending, more with the basket hook); STS3215: the **12 V C018**.",
    "Aluminium HTD 5M pulleys: 15 mm belt, **12 mm bore** (many default to 8 mm), two set screws.",
    "Oldham couplings: **12 mm clamp-style** hub, OD ≤ Ø42 (the cartridge nose bore is Ø43); measure the disc (thickness, slot width and "
    "depth) and set `OLDHAM` in rnd/parts/cartridge_m.py before printing the tongue adapters.",
    "Heat-set inserts: the short M4 (5.7 mm) the flange calls for; the long 8.1 mm ones don't fit the 6 mm flange.",
    "Belts: closed HTD **5M**, 15 mm wide (± 5 mm is taken up by the eccentric bushings).",
]


def totals():
    now = line_sum(items())
    a_yes, a_no = line_sum(HOLD_A["yes"]), line_sum(HOLD_A["no"])
    b_yes, b_no = line_sum(HOLD_B["yes"]), line_sum(HOLD_B["no"])
    c = 1 + CONTINGENCY
    return dict(now=now, now_c=now * c, a_yes=a_yes, a_no=a_no, b_yes=b_yes, b_no=b_no, d_a=(a_yes - a_no) * c, d_b=(b_yes - b_no) * c,
                d_a_hi=HOLD_A["hi"]() * c, d_b_hi=HOLD_B["hi"]() * c,
                default=(now + a_no + b_no) * c, a_only=(now + a_yes + b_no) * c, both=(now + a_yes + b_yes) * c)


def _table(L, rows):
    L += ["| Group | Item | Qty | Unit $ | Line $ | Why | Price source |", "|---|---|---|---|---|---|---|"]
    L += [f"| {g} | {n} | {q} | {u:,.2f} | {q * u:,.0f} | {w} | {s} |" for g, n, q, u, w, s in rows]


def main():
    I = items()
    t = totals()
    c = CONTINGENCY
    n_est = sum(1 for *_, s in I if s.startswith("ESTIMATE") or s == E)
    L = ["# Milestone M1: one-arm bench kit (generated by `python -m rnd.m1_kit`)", "",
         f"**Buy now: ${t['now']:,.0f} + {c:.0%} contingency = ${t['now_c']:,.0f}.** "
         f"Hold A (GIM8108-8 at pitch + elbow): **{sd(t['d_a'])}** vs the GIM6010 default (up to {sd(t['d_a_hi'])} at the dearest listing). "
         f"Hold B (inline GIM8108-36 roll): **{sd(t['d_b'])}** vs the belt roll (up to {sd(t['d_b_hi'])}). All with contingency.", "",
         "| Scenario (with contingency) | M1 kit |", "|---|---|",
         f"| Buy now only (yaw, wrist and everything else; pitch / roll / elbow motors not yet) | ${t['now_c']:,.0f} |",
         f"| Both holds answered no (today's params: 4 × GIM6010-8, belt roll) | ${t['default']:,.0f} |",
         f"| Hold A yes, hold B no (D20) | ${t['a_only']:,.0f} |",
         f"| Hold A no, hold B yes (**today's plan**: GIM6010 at pitch + elbow, approved inline roll) | ${t['default'] + t['d_b']:,.0f} |",
         f"| Both yes (D20 + inline roll) | ${t['both']:,.0f} |", "",
         f"{len(I) - n_est} of the {len(I)} buy-now lines are priced from retailer snapshots, {n_est} are ESTIMATE. Everything except the bench "
         "supply and the catch is reused in the full robot.", "",
         "**Zero-regret shortcut:** if you want all four GIM joints turning on day 1 before the seller answers, buy the GIM6010-8s of both "
         "holds' *no* lists now (+3 × $90): the full robot needs at least 7 of them anyway (4 wheels, lift, 2 yaws), so a GIM6010 that a "
         "hold later replaces moves to the base. The GIM8108s are the only parts that wait.", "",
         "**What M1 proves on real hardware:** joint strength, belts and the D13 v3 stages with the Oldham drive; the bolted cartridges "
         "(flange screws, centring lip, Oldham lash); the balancer (A9); hand accuracy with the joint encoders (≤ 1 mm target) and the "
         "compliance chain (rnd/stiffness.py); the fin-ray grasp and the D16 wrist spindle; motor and can temperatures (A6), CAN timing "
         "(SW1, A14), e-stop and watchdog time (SW2) with sw/bringup.py.", "",
         "## 1. Buy now (decided, no open question)", ""]
    _table(L, I)
    L += [f"| | **Subtotal** | | | **{t['now']:,.0f}** | | |", f"| | Contingency {c:.0%} (A16) | | | {t['now'] * c:,.0f} | | |",
          f"| | **Total** | | | **{t['now_c']:,.0f}** | | |", ""]
    for k, H in (("a", HOLD_A), ("b", HOLD_B)):
        y, n = t[f"{k}_yes"], t[f"{k}_no"]
        L += [f"## {'2' if k == 'a' else '3'}. {H['title']}", "", f"**Waits on:** {H['waits']}.", "",
              f"**If yes, buy** (${y:,.0f}):", ""]
        _table(L, H["yes"])
        L += ["", f"**If no, buy instead** (the default lines this hold replaces; ${n:,.0f}):", ""]
        _table(L, H["no"])
        L += ["", f"**Delta: {sd((y - n))} before contingency, {sd(t[f'd_{k}'])} with it** "
              f"(at the dearest listing {sd(t[f'd_{k}_hi'])}). {H['note']}", ""]
    L += ["## 4. Questions for the seller (send as one message, before paying for any GIM)", ""]
    L += [f"{i}. {q}" for i, q in enumerate(SELLER_QUESTIONS, 1)]
    L += ["", "Ask Damiao (or the DM-J4310 reseller) separately: the V1.1 output bearing's allowable radial load and moment.", "",
          "## What changed since the loop-40 list", ""] + [f"- {x}" for x in CHANGES] + [
          "", "## Check before paying / on arrival", ""] + [f"- {x}" for x in CHECKS] + [
          "", "**Order of work:** the coupons ([COUPONS.md](COUPONS.md); C1, the D15 socket, now needs re-scoping to the bolted flange + "
          "Oldham tongue) can run once the P2S arrives; nothing bought here depends on a coupon result. Readiness and the day-1 plan: [M1_READINESS.md](M1_READINESS.md). "
          "Buying is the owner's decision."]
    (DOCS / "M1_BENCH_ARM.md").write_text("\n".join(L) + "\n")
    print("\n".join(L[2:11]))
    return t


if __name__ == "__main__":
    main()
