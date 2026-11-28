# Fable design review #2: Pip v4, before the M1 Step-1 purchase (2026-11-28)

Scope: tree at `b7922b3`. I re-read my first review (`fable_review_2026-11-27.md`, tree `280715d`), the plan, scorecard, both M1 docs and Astra #3, then the source: `rnd/parts/cartridge_m.py`, `shoulder.py`, `roll.py`, `encoder.py`, `rnd/electrical.py`, `rnd/safety_board_sim.py` (logic model), `rnd/joints.py` output, `sw/safety.py`, `sw/bringup.py`, `sw/soak.py`, `sw/sysid.py`, `sw/rehearse_step1.py`, `sw/gim_discover.py`, `sw/config/*`. One light re-run (`python -m rnd.belts`, 6 s, output unchanged). No FEA, no `run_all`, no software test suites. Not reviewed in depth: the base category-1 stop, the hand-speed cap, the power-off and A10 studies, R17 covers (none of them is on the M1 bench).

**V** = verified in code, in a generated report, or by a run. **S** = suspicion from engineering judgement.

## Verdict

1. **Buy with changes: split Step 1 and buy only the first half now.** The $2,835 Step-1 kit is right in content but wrong in order.
2. **Step 1a, buy now (≈ $625, ≈ $720 with contingency):** the GIM6010-8, the DM-J4310, the two CAN adapters and the HAT, the 10 A CC supply, stand, lever stock, thermistors, inserts, screws, filament. It answers the one question everything else hangs on (the SDC104 protocol), on a bare, unloaded motor at a 1 A limit, which needs no safety board.
3. **Step 1b, buy later (≈ $1,840, ≈ $2,115 with contingency):** the SE-1000-24, all safety-board parts and the test instruments. Buy these after (a) the discovery result, (b) the board is drawn in KiCad and reviewed, and (c) the Step-1 rig is designed (C1 below). The repo already says the board parts wait for the schematic review; the GO line and the kit table do not follow that.
4. **Do not put the lever on the motor as the rig is specified today.** The rig has no hard stops and no guard, and the software is configured for a different machine (C1).
5. Step 2 stays no-go, correctly. Most of my first review's CAD findings are fixed on paper; what is left is listed in the follow-up.

Before spending anything: open the public GIM6010-8 manual (the URL is already in `rnd/specs/actuators.yaml:26`; this environment could not reach it, the owner can) and send the six seller questions. Both are free and could change Step 1a.

## Follow-up on review #1

| # | Finding | Status | Evidence |
|---|---|---|---|
| C1 | Arm hung from M3 inserts in pull-out | **Partly fixed** (V) | Tie-rod joints with steel crush sleeves at the pitch hub and yaw flange (`shoulder.py:95` HUB_TIE, `roll.py:821-839` YAW_TIE); `joints.md` gives 1.18–2.94× over four load cases. Open: `joints.md` itself lists the upper bone ↔ yaw flange screws, the detach clamp, the elbow core/clamp and the wrist output bearing as "not drawn yet" in the same wrench chain. No insert pull-out coupon exists (M4 below). |
| C2 | Compliant hold dropped the heartbeat | **Fixed** (V) | `sw/safety.py:675-678` keeps the heartbeat when the only latched fault is `collision`; `sw/test_s1.py:397` asserts it; `bringup.supervisor()` now takes `collision=` (`bringup.py:368`). |
| C3 | Cartridge could not react torque or be swapped | **Partly fixed** (V) | Rear cap v2 with dog ears and lug screws (`cartridge_m.py:362-382`); four through-bolted ears with nuts, no inserts in pull-out; R14 (c)/(d) are now real probes. Open: which face carries the GIM6010 housing holes (seller question 6). The probe now fails 5 of 6 joints on its own Ø12 × 50 mm rule (`assembly_seq.md`: every ear screw is blocked by its own can), and a pitch swap needs the bracket wall, hub, roll stage and arm off first. A plain 3 mm hex key does reach the ears, so this is a rule-versus-design mismatch to settle, not a dead end. The scorecard row still reads "R14 7/7 ✅". |
| M1 | Inline roll: preloads, stops, backlash | **Partly fixed** (V) | Metal rear clamp, steel stop bushes, bonded dowels, backlash in the accuracy model (4.3 mm without joint encoders). Open, all on the seller: the boss is 1.03× stall only with bonded dowels and an assumed cast-alloy allowable; the motor bearing needs ≥ 1,417 N in service against a 1,000 N listing (0.7×); SDC303 protocol. |
| M2 | Bench power and safety electronics | **Partly fixed** (V) | SE-1000-24 replaces the 10 A supply; one-shot discharge gated against pre-charge; watchdog 8 → 25 ms on a measurement (x86 only); netlist v3 simulated in ngspice with six changes. Open: still a netlist in prose, no schematic in the tree, not built (M2 below). |
| M3 | No encoder mounts, no node firmware | **Partly fixed** (V) | Mounts for all four joints in `rnd/parts/encoder.py`. Open: no RP2040 node firmware anywhere in `sw/`, and no USB-TTL servo adapter in the kit, so in Step 2 the wrist roll and gripper cannot move. `M1_READINESS.md` still says the mounts do not exist. |
| M4 | GIM protocol unknown | **Open, by design** | Step 1 exists to answer it; `sw/drivers/gim.py` is still a TODO table. Nothing more can be done without the motor or the manual. |
| M5 | Belt adjusters used up; friction drives | **Fixed** (V) | D26 removed the Oldhams and eccentrics; centres 81.48 / 71.21 with a tension ring; integral hobbed pinion on 8 × M3 (2.47×). New risk from the fix: M3 below. |
| M6 | Hold A was not "motors only" | **Fixed as a plan** (V) | D20 has its own parts list and is on hold. `belts.md` still shows the D20 ear friction at 0.69× of stall with 30 % preload. |
| M7 | Thermal margins optimistic | **Partly fixed** (V) | Network model: closed sleeve 0.65× rated, pitch winding 103 °C on a task day, 60 °C with an aluminium cap. The cap depends on seller question 6. Thermistors are in Step 1. |
| M8 | Balancer pulleys overloaded | **Fixed** (V) | 608-2Z in POM tyres, D/d ≈ 12 (`balancer.py:31-39`). |
| minor | R17 not on the scorecard | **Fixed** | Now a hard gate, failing. The scorecard shows 16 / 20; `pinch.json` says 13 / 6 (stale). |
| minor | Readiness doc stale | **Partly** | Regenerated, with new stale lines (see Minor). |
| minor | Custom metal parts, stand, button heads, regen table | **Fixed** (V) | All in the kit or the netlist. |

## Critical

### C1. The Step-1 lever rig is undesigned, unguarded, and the software describes a different machine (V)

- **What is bought:** a 300 mm aluminium bar "bolted straight to the GIM output" with 1 kg and 2 kg weights, on two extrusions (`M1_BENCH_ARM.md`, Step-1 rig lines). No belt stage.
- **What the software runs:** joint `yaw`, which is `ratio: 2.0` in `sw/config/can_map.yaml:14` with `rated_nm: 10`, `peak_nm: 22` in `sw/description/joints.json`. There is no Step-1 rig description and no ratio override. The rehearsal that "passes" (`sw/rehearse_step1.py:6-7`) simulates the yaw stage turning the forearm in the arm fixture, not a lever on a bare motor.
- **Consequences on the real rig:**
  - Every commanded angle and speed is doubled at the lever, and every torque the supervisor believes in is 2× the real one.
  - `sw/soak.py:9` calls 2 kg at 0.3 m "≈ 59 % of the yaw stage's rated torque". On the bare motor it is 5.9 N·m against 5 N·m rated: **118 % of rated for two hours**, on a drive whose flash limit is set to rated. It either cannot hold the weight or cooks. The 50 % load is 0.85 kg.
  - `calibrate` seeks a hard stop at +155° and checks the span to −155° (`bringup.yaml`, `bringup.py:831-848`). The rig has no stops; the yaw stop pin is a Step-2 line. The step fails, or with the ratio error sweeps the lever ±310° with the weight on.
  - Gravity feed-forward and the sysid torque-constant fit use the twin's forearm and grasp point, not a 0.3 m bar (`sysid.py:290-313`). Exit criteria 3, 5 and 7 cannot be met as written.
- **Why it is a safety item:** a bare GIM output turns without limit. At the rated 5 N·m a 2 kg lever gains about 31 J per revolution; at the drive's 420 rpm the weight is at 13 m/s and needs 1.2 kN to stay on the bar. The catch cradle and tether are sized for a 50 mm drop, not for a rotating bar. The first loaded enable is also the first time an unproven protocol table, sign and limit are trusted.
- **No CAD:** there is no fixture for the motor, the DM-J4310 or its load, and no guard (a search of `rnd/` and `sw/` finds none).
- **Fix before the lever is fitted (no purchase needed):**
  1. A `step1` rig entry: ratio 1.0, rated 5 / peak 11 N·m, the lever and weight as the gravity model, limits ±90°.
  2. Two steel hard stops on the motor plate at about ±100°, and a motor plate drawn and checked for 11 N·m plus a stop impact.
  3. Weights bolted through the bar, not hung. A polycarbonate shield or a marked keep-out wider than the lever circle.
  4. Soak mass 0.85 kg; first loaded enable with the bare bar, the supply at 2 A and the drive speed limit set.
  5. A rehearsal that models this rig, so a pass means something.

## Major

### M1. Step 1 buys $1.84k of board, supply and instruments before the two facts that decide them (V)

- Of the $2,465 subtotal, about $625 is the motors, CAN, small supply, stand and print stock. The rest is the SE-1000-24, the safety-board lines and $1,216 of instruments.
- `gim_discover` runs on one unloaded motor at a 1 A limit (`sw/gim_discover.py:17-19`). That needs none of the $1.84k. The day-1 plan nevertheless puts "build the safety board" first.
- If the SDC104 has no impedance mode, no CAN timeout or no flash limit, the repo's own answer is "stop and redesign the safety argument". The board's timing budget and the bench plan are part of that argument.
- `M1_BENCH_ARM.md` says the logic lines are ordered "after the KiCad schematic is drawn and the checklist is signed", yet the clamp, contactor and instrument lines sit in "buy now" and the readiness line reads GO.
- **Fix:** Step 1a / 1b as in the verdict. In 1a also prove on the bare output: enable, zero-gain, impedance, the CAN timeout, the flash limit over a power cycle, and the two measurements in M3.

### M2. The safety board is still a first-time build of an undrawn design, and its logic has an unresolved race (V netlist and model, S hardware behaviour)

- **Not drawn:** no KiCad, SPICE or schematic file is in the tree. The gate needs "someone who did not draw the board" to sign; no such person is named. For a one-person project this gate can stall Step 1 indefinitely.
- **Race on a K1 drop-out the logic did not command** (coil wire, shock, coil dip). `K2_DRIVE = LATCH ∧ ¬AUX` and U6's `CLR̄ = ¬K2_DRIVE`. When AUX falls with LATCH still high, the same edge triggers U6 and, two gate delays later, clears it. `electrical.md` says this opening "clears LATCH so K2 cannot re-close by itself". The repo's own logic model says the opposite: `safety_board_sim.py:690` fires FORCE only if `not k2_drive`, and the "AUX wire breaks" row reports "no FORCE … K2 re-closes" as drawn, and FORCE only at +30 ms with the discrepancy fix. On hardware the outcome is a few-nanosecond FORCE glitch into the latch's clear pin, which may or may not take.
- **The test sheet contradicts the model:** T7 demands FORCE "on every opening … by unplugging the K1 coil" and the far end under 10 V within 4 ms of AUX. As designed that case gives +30 ms and a pre-charge re-close in between. Either T7 fails or the design changes.
- **No debounce on AUX** (S). AUX comes from a mechanical contact through a PC817 and feeds an edge-triggered one-shot and the latch clear. A bounce at K1 closing produces a falling edge with LATCH high: FORCE, LATCH cleared, and the clamp across a still-closed K1 for the release time, which latches the SE-1000 off. Expect "arms most of the time".
- **Smaller:** the checklist's U10 clear terms (`STOP ∨ FORCE ∨ POR ∨ PRE_TIMEOUT`) and the netlist table's (`STOP ∨ FORCE ∨ POR ∨ DISC ∨ UV12 ∨ OC`) differ.
- **Fix:** debounce AUX (RC + Schmitt, about 1 ms); clear LATCH directly from "Q4 commanded on and AUX low" instead of through the one-shot; add the coil-open case to the logic model. Better (S): let a bought safety relay with monitored reset and contactor feedback do the e-stop, reset and welded-contact jobs (the design already uses a G9SA for the base), and keep the custom board for the clamp, pre-charge and watchdog. That removes about ten single-gate ICs and most of the review burden.

### M3. The D26 direct stage rests on an unmeasured property of the motor (V numbers, S on the motor)

- The pinion shaft is now carried by the motor's output bearing and a bonded 6001 in wall B: three supports on one shaft. `belts.md` shows it passes only if the motor's output tilts freely ("hinged": fatigue 3.13×). If the output is moment-rigid, a 0.15 mm offset after a motor swap gives fatigue **0.56×** and 2.0 kN plus 110 N·m on the motor bearing.
- A planetary carrier supported on two bearings is common (S). The only bearing figure is a reseller's "800 N radial".
- Belt tension is set once by a 0.30 mm ring "before the 6001 bond". There is no re-tension without breaking the bond, and housing creep under the sustained belt pull is not in the 134–220 N band (0.25 mm of centre distance is the whole preload) (S).
- The one-off hobbed C45+QT pinion is priced at $35 "each in 10s". A single piece is more likely $100 or more (S).
- **Fix:** in Step 1a measure the output's tilt stiffness (side load on a bar, dial indicator) and its bearing layout, with the PCD and pilot. Quote the pinion only after that. If the output is stiff, use the overhung variant the repo already names (`STUB_SUPPORT_RULE`) and drop the 6001.

### M4. Step-1 exit criterion 6 has no coupons behind it (V)

- The criterion needs an insert pull-out coupon, the bolted 4-ear flange coupon and the drive-stub coupon. `docs/COUPONS.md` and `rnd/coupons.py` contain none of them. They still list the KX socket (C1) and the eccentric bushing (C5), which D24 and D26 removed.
- `joints.md` adds five more "coupons to add" (sleeved joint moment to gap, creep at 40 °C, stop bush, dowel slip, 7075 threads). None is defined.
- **Fix:** write the three Step-1 coupons with geometry, load and pass number before printing anything; delete the obsolete ones.

### M5. The rest of the arm's load chain is not drawn (V)

- C1 fixed two joints. `joints.md` says the upper bone ↔ yaw flange screws, the detach clamp, the elbow core and clamp, and the wrist output bearing "stay open" and are not in CAD.
- The same 130–184 N·m fault wrench passes through them. A chain is as strong as the joint nobody has checked.
- **Fix:** a Step-2 gate, next to C3: every joint between the shoulder and the hand has a row in `joints.md`.

## Minor

- **Scorecard overstates twice (V).** R14 "7/7 ✅" while the R14 (c) probe fails at pitch, elbow, yaw, lift and roll; R17 "16 / 20" while `pinch.json` has 13 / 6.
- **`M1_READINESS.md` stale lines (V).** Encoder mounts "do not exist"; C1/C3/M5 CAD fixes listed as still to land; seller question 1 quotes the clamp at 26.4 / 27.3 V, the board is 26.0 / 25.7 V.
- **Differential probe, $175, is not needed (V).** Q1–Q3 and Q4 are low-side switches; their drains are measured to board ground with a ×10 probe and a ground spring. The SE-1000 output floats.
- **NH-50 resistors are a lead-time risk for little gain (S).** No 0.56 Ω price was found and one listing shows 10 weeks. With the freewheel Schottky across each resistor, a stock wirewound RH-50 is acceptable: its L/R is far below the 50 µs minimum on-time, and the drain overshoot is set by the board loop, not the resistor.
- **Set the bench supply to 24.0 V, not 25.2 V (S).** The clamp's lowest turn-off is 25.44 V. At 25.2 V the margin is 0.24 V against the supply's ±1 % and 150 mV ripple; a drift makes the clamp chop against the supply until the thermal switch trips.
- **Watchdog margin is proven on a 4-core x86 only (V).** Criterion 4 covers the Pi. The soak's "jitter p99 < 1 ms" in Python over USB-CAN is likely to fail first (S); decide beforehand whether that blocks Step 2.
- **Crush sleeves faced by the builder to ± 0.02 mm (S).** PETG moves 0.05 mm over 70 mm for 10 K. Buy them turned to a drawing, as the kit prices them, and drop the "builder cuts them" option.
- **Thermistors have no readout in the kit (S).** Ten NTC beads and no ADC; the DMM thermocouple covers Step 1 by hand.

## What's solid

- **The follow-through.** Eleven of my thirteen findings were acted on within a day, most with a calculation that reports its own "before" as failing. C2 is fixed with a test that names it.
- **Honest reports.** `assembly_seq.md` fails its own design on tool access; `belts.md` prints the 0.56× bound; the board simulation lists four faults in the netlist it was asked to confirm; R17 is a failing hard gate.
- **C1 as engineered.** Metal-to-metal preload paths, creep modelled instead of a flat factor, contact faces checked per material.
- **The software core.** Supervisor, discovery tool, bring-up sequence, the gated steps, soak and sysid definitions. The problem in C1 is configuration, not code quality.
- **Buy without regret (Step 1a):** the GIM6010-8, the DM-J4310, CAN adapters, the 10 A supply, extrusion, bar stock, inserts, screws, filament.
