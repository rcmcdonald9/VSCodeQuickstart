# Safety board in KiCad (netlist v4 → schematic + board, for review gate G-SB)

**What this is:** the electrical.md *Safety board* netlist (v4, rnd/electrical.py) drawn as a real KiCad 7 project in
[hardware/safety_board/](../../hardware/safety_board/), generated from the netlist source so the drawing cannot drift from the
analysed design. It is the "board drawn in KiCad" that RND_PLAN §4 item 5 and the G-SB gate ([safety_board_tests.md](safety_board_tests.md))
wait for. **Schematic: complete, ERC clean. PCB: placed with power pours, not routed.** Not a certified safety function.

| | Result |
|---|---|
| Schematic | 6 sheets (root + Power, Clamp, Logic, Base, Connectors), 302 parts (141 from netlist v4, 161 added by the drawing, each with its reason), 153 nets, every IC pin connected or flagged |
| KiCad ERC (eeschema 7.0.11) | **0 errors, 0 warnings** ([safety_board_erc_kicad.rpt](../../hardware/safety_board/safety_board_erc_kicad.rpt); screenshot `out/erc_gui_2_result.png`) |
| Netlist ERC (KiCad pin matrix on the exported netlist) | 0 errors, 0 warnings ([safety_board_erc.txt](../../hardware/safety_board/safety_board_erc.txt)) |
| Cross-check drawing ↔ model ↔ electrical.py | 153/153 nets identical node-for-node; 43/43 netlist rows drawn, 118/118 named nets on the named parts; 15/15 block-diagram chains ([safety_board_crosscheck.txt](../../hardware/safety_board/safety_board_crosscheck.txt)) |
| PCB | 240 × 135 mm, 2 layers, 2 oz (70 µm) copper, 288 footprints placed, power pours filled; **signal routing not done** |
| KiCad DRC (pcbnew 7.0.11) | **0 rule errors** (clearance, courtyard, mask, edge); 499 unconnected pads (= the unrouted board); warnings: 39 silk overlaps, 2 silk-to-edge, 199 "footprint not found in library" (headless library-table artefact, below) ([safety_board_drc.rpt](../../hardware/safety_board/safety_board_drc.rpt)) |
| Exports | [schematic PDF](../../hardware/safety_board/safety_board_schematic.pdf), [board netlist](../../hardware/safety_board/safety_board.net) (for the PCB), [full netlist with the chassis parts](../../hardware/safety_board/safety_board_full_netlist.xml), [BOM CSV](../../hardware/safety_board/safety_board_bom.csv) (185 lines, with Location / Variant / Origin / Netlist ref), [PCB front](../../hardware/safety_board/safety_board_pcb_front.pdf) / [back](../../hardware/safety_board/safety_board_pcb_back.pdf) |

## Tooling and how to reproduce

- **KiCad 7.0.11** from Ubuntu noble (`apt-get install -y kicad`; pulls kicad-symbols / kicad-footprints 7.0.11 and `kicad-cli`). SKiDL was not used: the generator writes KiCad's S-expression files directly (no extra dependency, exact control of symbol, footprint and fields).
- **kicad-cli 7.0.11 has no `sch erc` and no `pcb drc`** (both arrived in KiCad 8.0; the KiCad 8 PPA and Flathub are blocked from this environment). So:
  ERC = eeschema's own ERC dialog driven headless in a virtual X server (`apt-get install -y xvfb xdotool imagemagick`, `python -m rnd.kicad_erc_gui`), plus a netlist-level ERC with KiCad's default pin-conflict matrix (rnd/kicad_check.py) that runs on every generation;
  DRC = `pcbnew.WriteDRCReport` from KiCad's python module (rnd/kicad_pcb.py, system `/usr/bin/python3`, which has `pcbnew`). On KiCad 8+ use `kicad-cli sch erc` / `kicad-cli pcb drc` on the same files.
- First run of pcbnew on a fresh machine: `cp /usr/share/kicad/template/fp-lib-table ~/.config/kicad/7.0/` (eeschema's first-run dialog creates the symbol table; rnd/kicad_erc_gui.py clicks it).

```
python -m rnd.kicad_board --pcb      # schematic + project + netlists + PDF + BOM + cross-check + netlist ERC, then the PCB + DRC
python -m rnd.kicad_erc_gui          # KiCad's own ERC (eeschema under Xvfb) → safety_board_erc_kicad.rpt
python -m rnd.kicad_check            # cross-check only (after a generation)
```

Files: rnd/kicad_board.py (design model + schematic writer), rnd/kicad_symbols.py (project symbol library), rnd/kicad_sexp.py (S-expression reader / writer, library flattening), rnd/kicad_check.py (cross-check + netlist ERC), rnd/kicad_pcb.py (board), rnd/kicad_erc_gui.py (GUI ERC). **Do not edit the .kicad_sch files by hand**: change electrical.py (or the drawing layer in kicad_board.py) and regenerate. If electrical.py's table changes in a way the drawing assumed (a part, a net, a value with no constant), `need()` stops the generator with the row that drifted.

## How the drawing follows electrical.py

- **Parts**: every ref in electrical.md's ref table and block diagram is a symbol; the KiCad field `NetlistRef` gives the electrical.py ref (block) it implements, `Origin` says `netlist v4` or `added: <why>`, `Location` board / chassis, `Variant` all / robot (base channel, not fitted on the M1 bench) / M1, `Basis` where its value comes from.
- **Values** come from electrical.py's constants where they exist (watchdog R4 from `bringup.yaml watchdog_s`, one-shots from `T_FORCE` / `T_ON_MIN`, debounce from `AUX_DEBOUNCE`, clamp divider from `CLAMP` and `CMP_FILTER`, timers from `T_PRE_TIMEOUT` / `DISC_WINDOW`, `R_PRE`, `R_BLEED`, `C_BULK`, fuses), otherwise from the table text (checked). Resistors are E96, the comparator divider 0.1 %:

| Quantity | electrical.py | Drawn |
|---|---|---|
| clamp on / off (U4, U23) | 26.00 / 25.70 V | 25.98 / 25.68 V (100k / 10.7k-class E96 + 1.1 M hysteresis, TL431B 2.495 V): inside the ± 1 % the simulation already carries |
| comparator filter | 2.0 µs | 2.14 µs (11.9 kΩ Thevenin × 180 pF C0G) |
| PRE_OK | VBUS ≥ VIN − 3 V | VBUS ≥ VIN − 3.03 V (offset network from VREF; − input 0.0991·VIN) |
| watchdog U5 / U22 | 25 ms | 25.0 ms (250 k × 100 nF C0G) |
| min on-time U16 / U24 | 50 µs | 50.3 µs (10.7 k × 4.7 nF C0G) |
| FORCE U6 / FORCE_B U25 | 100 ms | 100 ms (100 k × 1 µF) |
| pre-charge timeout U11 | 1.0 s | 1.01 s (1 M × 1 µF to 0.635·VDD) |
| discrepancy window U17 | 40 ms ± 10 % | 39.5 ms (39.2 k × 1 µF) |
| DISC_B (U26) | 50 ms | 50.3 ms |
| 12 V UVLO U18 | 10.5 V | 10.56 V (75 k / 10 k on the TLV3012's 1.242 V) |
| arm overload U19 | 16 A, 5 s | 16.2 A, 5.0 s (66 mV/A ESTIMATE) |
| debounce, R3/R5, bleeders, C1, NH-50 | as electrical.py | identical |

`python -m rnd.safety_board_sim` was not re-run: every drawn value above is the electrical.py value or inside the tolerance band the simulation already applies (± 1 % thresholds, ± 10–20 % timers). The G-SB reviewer re-runs it anyway (gate evidence item 5).

- **Pin level.** electrical.md names functions and nets, not pins. The drawing adds what a buildable circuit needs and marks it: 16 glue gate ICs for the boolean terms (U10 D and CLR̄, K2_DRIVE, DISC → U6 trigger, the base clamp OR and DISC_B AND), gate pull-downs and series resistors, decoupling (one 100 nF per IC), the comparator dividers, the LM5069 support parts, the studs and harness connectors for the chassis parts, test points.
- **Chassis parts** (F1, K1, R1–R3 / R7 NH-50, R5 pre-charge RH-50, S1, S2, SB1, TH1, U20, the supply cable J1, TP2 at the dummy bank) are drawn with `on_board no`: they are in the schematic, the full netlist and the BOM, and reach the board through M5 / M4 studs (10 AWG lugs) and keyed Micro-Fit / JST-VH harnesses.
- **Net names = electrical.py names** (all nets are global labels): VIN, VBUS, VBUS_B, COIL+ / COIL-, AUX_A / AUX_B / AUX_RAW / AUX_DB, LATCH, RUN, K2_DRIVE, PRE_OK, OV, FORCE, FORCE_B, WD_OK, WD_B, DISC, DISC_B, UV12, OC, PRE_TIMEOUT, U8_OUT, K2_C, F1_OUT, GPIO17 / 22 / 23 / 24 / 27. Two drawing-only nets: **PGND** (power return) joins **GND** (logic) only at net tie NT2 at C1−, and **VBUS_SNS** is Kelvin-tied to VBUS at NT1 (checklist layout items).

## Sheets

| Sheet | Contents |
|---|---|
| Power | supply → F1 → ideal diode U1/Q6 → VIN; K1 (chassis) and its coil driver Q4 (100 V, 12 V gate from U9 OUTB), D1 + Z1; K2 + R3 pre-charge (drawn R5), Q5; C1, NT1, NT2; arm branch U19 + F3; logic supply F2 → U2 → U3 (+ U32 5 V); F5 → +24V_L |
| Clamp | R1–R3 (chassis) ∥ D4–D6, Q1–Q3, R_G, pull-downs, C2, D3, R6; U7 OR3 → U9; U16; U4 (OV, PRE_OK) + TL431B |
| Logic | U30 (U5a) + U5 watchdog; S1b·S2b chain + U13; aux opto U12 → U58 → AUX_RAW → debounce U29 → AUX_DB; STOP / ARM / CLR̄ glue; SB1 debounce; U10 LATCH, U28 RUN; K2_DRIVE, U11; U15, U52, U6 FORCE; U8 + U57; U17 + U51 DISC; U18 UV12; U19's comparator U53 (OC) |
| Base (robot only) | U20 + U27; Q9 / U21 LM5069-1 + shunt; F4; R7 ∥ D7, Q10, C4, R8; U23, U24, U25, U56 → U31 (U22b); U22 WD_B; U26 DISC_B; Q11 caster solenoid driver |
| Connectors | K1 coil/aux J13; mushroom harnesses J14 / J15 and S1, S2 (4 NC each); ARM J16 + SB1; TH1 J17; U20 harness J19; Pi J18; mounting holes |

**Test points** (safety_board_tests.md): TP1 VBUS, TP2 VBUS at the dummy bank (off-board probe point on ARM_LEAD), TP3 Q1 drain, TP4 Q1 gate, TP5 WD_OK, TP6 Q4 gate, TP7 Q4 drain, TP8 AUX_RAW, TP9 FORCE, TP10 LATCH, TP11 K2_DRIVE, TP12 AUX_DB, TP13 RUN. Added: TP14 +12 V and TP15 +3V3 (T0), TP16 VBUS_B, TP17 Q9 enable, TP18 Q10 gate, TP19 FORCE_B, TP20 DISC_B, TP21 WD_B (T17–T19), TP22 PRE_OK, TP23 OV, TP24 PRE_TIMEOUT, TP25 DISC, and scope-ground points TP26 GND, TP27 PGND at Q1, TP29 PGND at Q4.

## Netlist issues found (to fix in rnd/electrical.py, not fixed there)

| # | Issue | How the drawing handles it |
|---|---|---|
| N1 | **Duplicate ref R3**: "R1–R3 0.56 Ω NH-50" (clamp) and "R3 Vishay RH-50 3.3 Ω" (pre-charge) | clamp keeps R1–R3; the pre-charge resistor is **R5** (NetlistRef "R3") |
| N2 | **+24V_L has no source** (coil chain, K2 coil, aux loop, U20) and U2's row says nothing else may hang on F2 | **F5** 2 A from VIN → +24V_L (added) |
| N3 | **U19 ACS724 is a 5 V part**; the netlist has 12 V and 3.3 V only | **U32** MCP1703A-5002 (added); output halved into the 3.3 V comparator |
| N4 | **U7 is listed as SN74LVC1G32 (2 inputs) but must OR three terms** (OV, FORCE, U16's on-time) | drawn as **74LVC1G332** (3-input OR) |
| N5 | **Q5 = IRLML6344 (30 V)** while design change 1 says Q5 ≥ 60 V; **K2's coil has no suppression** in the netlist (checklist asks for it) | Q5 drawn as listed (flagged); **D9** flyback across K2's coil added |
| N6 | Parts with no ref: the base fuse "F 20A", Q11's flyback diode, the R/C of U6 / U16 / U22 / U24 / U25, the base leg's gate resistor | F4, D8, R67/C51, R52/C46, R96/C62, R94/C59, R95/C60, R14 |
| N7 | **Base clamp leg has no local film capacitor** (the arm leg's C2 closes the 30 nH MOSFET–diode loop) | **C4** 10 µF film on VBUS_B (added) |
| N8 | Caster-release gate "from the Pi": no GPIO named | J18 pin 6 `CASTER_REL` (assign a GPIO in sw/) |
| N9 | **SB1 must be debounced** (checklist) but the netlist has no parts for it | R62 / R63 / C49 + U59 Schmitt (τ 1 ms) |
| N10 | Q4 "gate U8_OUT" (3.3 V) but IRL540N is not specified at 3.3 V; design change 1 says drive it from the UCC27524's spare channel at 12 V. Then U17's "XOR(Q4 gate, …)" input would be 12 V on a 5.5 V part | Q4 driven by **U9 OUTB** from U8_OUT; U17 reads the logic-level command U8_OUT |
| N11 | U10's D and CLR̄, K2_DRIVE and "DISC → U6 trigger" are boolean terms with no gates allocated (≈ 10 single-gate ICs, as route C notes) | glue U40, U41, U43, U44, U46, U47, U52, U60–U63 (+ U58, U59 above); CLR̄ = gate output wired-AND with the R9·C9 POR through D21 |
| N12 | The debounce model assumes a stiff AUX_RAW; an opto emitter with a pull-down would put ≈ 10 kΩ in series with R_d (fall time ≈ 3×) | PC817 collector + pull-up → **U58 74LVC1G14** → AUX_RAW (push-pull) |
| N13 | Minor: row "S1, S2" says 2 NC each, row "S1c/d, S2c/d" adds two more (4 each on the robot) | drawn with 4 NC blocks each |

## Reviewer: verify (pinouts and values the drawing could not check against a datasheet here)

- **Project-library pinouts** (pip_safety.kicad_sym, drawn from memory of the datasheets): TLV3202 (LM2903 body), 74LVC1G74 DCU, TLV3012 (SOT-23-6, open-drain, 1.242 V REF), UCC27524, UCC27517, LM5069-1 (MSOP-10), AQV252G, VS-40CPQ060 (1, 3 = A, 2 = K), G9SA-321-T terminal names, ALEV100 terminals. KiCad library parts used as-is (LM74700, PC817, G5LE-1 with NO = pin 4, ACS724, TL431DBZ, MCP1703, R-78B12).
- **LM5069 sense**: electrical.py's "1 mΩ sense" with "35 A breaker": the LM5069 current-limit threshold (≈ 55 mV, check) gives ≈ 55 A on 1 mΩ; 35 A needs ≈ 1.5 mΩ. Timer C56, power-limit R82 and the dv/dt soft-start (C55 / R81) are ESTIMATEs to size for Q9's SOA.
- U19 pin 6: FILTER per the KiCad symbol (1 nF fitted); if the chosen ACS724 variant has BW_SEL there, replace C53 by a link to GND. Sensitivity 66 mV/A ESTIMATE.
- TL431B decoupling C44 1 nF: check the TL431 stability boundary at ≈ 1.2 mA.
- R-78B12-2.0 chosen for U2 (32 V input) over the 28 V R-78E: the drive limit is 27.5 V.
- POR: R9·C9 gives ≥ 50 ms after 3.3 V; the checklist's "brown-out below 2.5 V clears it too" needs a supervisor (not in the netlist, not drawn).
- Checklist wording: with D1 + Z1 in series the Zener is reverse-biased only at turn-off (clamping); in normal operation D1 blocks and the Zener carries no current.

## PCB (stretch): placement, copper, DRC

- **Outline** 240 × 135 mm, 2 layers, **2 oz (70 µm) copper both sides**, 1.6 mm FR4, ENIG; 4 × M3 holes.
- **Placement**: one heatsink row along the top edge (Q6, Q9, Q10, D7, Q1, D4, Q2, D5, Q3, D6, Q4; one aluminium bar, insulated pads under the drain tabs); the 10 AWG studs straight under their devices (J2 F1_OUT, J4 VIN, J22/J23 base leg, J7–J9 DRN1–3, J10 NH-50 common, J5 VBUS, J3 PGND); C2 between the clamp legs, C1 at the right with the VBUS stud and the arm branch (U19, F3, J11/J12); K2 and the pre-charge terminal J6 in the middle; the logic packed below on its own GND plane (B.Cu), which meets PGND only at NT2.
- **Copper for the currents in electrical.md** (IPC-2221 external, 20 K rise, 70 µm): supply path 52 A (SE-1000 trip) needs ≈ 23 mm on one layer, the robot's 37.3 A sustained ≈ 14.5 mm; arm and base branches 20 A ≈ 6.1 mm; a clamp leg 9.4 A rms (150 W continuous over three legs) ≈ 2.2 mm, with 49 A pulses ≤ 0.4 s (adiabatic: not the sizing case). Poured: F1_OUT 16 × 22 mm and VIN 23 × 22 mm on both layers (≥ 2 × the 52 A width), VBUS 154 × 39 mm on both layers, VBUS_B 36 × 22 mm, the DRN pads 11–13 × 13–15 mm, PGND on B.Cu under the power stage and on F.Cu under the MOSFET sources; pads join the pours solidly (no thermal reliefs).
- **Not routed**: the logic, the gate drives and the drain / gate pins of the heatsink row (whose pads sit above the pours) are not connected; DRC lists them as 499 unconnected pads. Routing, the clamp loop (≤ 15 × 15 mm, ≤ 30 nH: C2 → J10 → NH-50 → J7–J9 → D/Q → PGND → C2) and the creepage gaps between VBUS and logic (≥ 1 mm) are the next step, then the checklist's layout lines.
- **DRC**: 0 clearance / courtyard / mask / edge errors. Warnings: 39 silkscreen overlaps and 2 silk-to-edge (cosmetic), 199 "footprint not found in library": the headless pcbnew used for DRC does not load the footprint library table (the footprints are the stock KiCad 7.0.11 ones, loaded by path); not re-checked in the pcbnew GUI.

## What the G-SB reviewer still needs

1. The routed board (the PCB here is a placement + pour start) and its DRC; the layout half of the checklist cannot be signed before it.
2. The electrical.py fixes N1–N13 adopted (or accepted) and the board regenerated; then a commit hash to review (gate evidence item 2).
3. The datasheet checks above (project-library pinouts, LM5069 sense value, ACS724 pin 6, TL431 stability).
4. Their own `python -m rnd.safety_board_sim` run on the drawn values (crosscheck section 5 lists them).
5. The owner's choice of route and reviewer (RND_PLAN §4 item 3); this drawing does not close the gate.
