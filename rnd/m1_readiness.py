"""M1 purchase readiness: what each step of the bench-arm buy depends on, how sure each answer is, the go / no-go per
step, the day-1 plan and the risks.  python -m rnd.m1_readiness

Reads the cached results of the checks (docs/rnd/*.json, docs/sw/*.json) so the numbers in the report are the ones the
checks produced; the kit totals come from rnd/m1_kit.py, the Step-1 coupon pass numbers from rnd/coupons.py. Structure after
Fable review #2 (2026-11-28): Step 0 (free), Step 1a (bare-motor discovery, buy now), the 1a → 1b gate, Step 1b (the loaded rig
and the safety chain), Step 2 (the rest of the arm), each with its own go / no-go.
"""
from __future__ import annotations

import json
from pathlib import Path

D = Path(__file__).resolve().parent.parent / "docs"
J = lambda n: json.loads((D / "rnd" / n).read_text()) if (D / "rnd" / n).exists() else {}
JS = lambda n: json.loads((D / "sw" / n).read_text()) if (D / "sw" / n).exists() else {}
V, C, E, U = "verified (sim / CAD check)", "measured in CAD", "ESTIMATE", "untested (M1 / coupon decides)"
nan = float("nan")


def main():
    from .m1_kit import CONTINGENCY, SELLER_QUESTIONS, STEP0, STEP1B_GATE, sd, step1, totals
    from .coupons import step1_coupons
    from .params import BASELINE
    from .spring import workspace_margins
    from . import accuracy, electrical as el
    wm = workspace_margins(BASELINE)
    t = totals()
    cp = {c["id"]: c for c in step1_coupons()}
    th, st, ri, ss, bom = J("thermal.json"), J("stiffness.json"), J("roll_inline.json"), J("slim_study.json"), J("bom.json")
    bt = bom.get("base_timer", {}).get("totals", {})
    bsel = bom.get("base_timer", {}).get("selected", "board")
    balt = next((k for k in bt if k != bsel), "g9sa")
    jit = JS("loop_jitter.json")
    mr = min(v["m_rated"] for v in wm.values())
    worst = min(wm.items(), key=lambda kv: kv[1]["rated"] / kv[1]["max"])
    acc_enc, acc_raw = accuracy.score(BASELINE)
    a_m1 = el.arm_currents(BASELINE, servos=True)
    st_m1 = el.stop_timing("M1")
    v1 = next((v for k, v in ss.items() if k.startswith("V1")), {})
    hug = v1.get("hug")
    n_est = sum(1 for *_, src in step1() if src.startswith("ESTIMATE"))
    wd = el.WATCHDOG["t_nom"] * 1000
    jmax = (jit.get("loaded") or jit).get("interval_ms", {}).get("max", nan)
    st_r = el.stop_timing("robot")
    el_tmax = 1000 * (el.SW2_BUDGET - (st_r["total"] - el.WATCHDOG["t_nom"] * (1 + el.WATCHDOG["tol"]))) / (1 + el.WATCHDOG["tol"])
    rows = [
        ("1a", "Motors: one GIM6010-8 and the DM-J4310 (the STS3215 pair is a Step-2 line: no 12 V rail before the elbow node's buck)",
         f"workspace margin ≥ {mr:.2f}× rated at p95 (1 kg), **worst pose {worst[1]['rated'] / worst[1]['max']:.2f}× ({worst[0]})** (spring.py)",
         V, "high (85 %)", "Unchanged by either hold. 1b measures torque constant and friction on the rig's lever."),
        ("1a", "GIM CAN protocol (SDC104)", "MIT mode named in the GIM6010-8 manual rev 1.2 and the GUI v2.0; no public CAN timeout, flash torque "
         "limit or status codes (docs/sw/gim_protocol_notes.md)", E, "low (40 %)",
         "**This is what 1a finds out** (`python -m sw.gim_discover`, rehearsed in sw/test_s2.py) on the bare output at 1 A. Read the public manual "
         "first (Step 0). If there is no impedance mode, CAN timeout or flash limit: 1b is not bought; the safety argument is redesigned first."),
        ("1a", "GIM6010-8 output bearing and pattern (C9, C14)", "the D26 stub assumes a single-row output bearing that tilts freely ('hinged', "
         "fatigue 3.13×); moment-rigid drives it to 0.56× at a 0.15 mm swap offset (belts.md); PCD 34 ESTIMATE", E, "medium (50 %)",
         f"C14 measures it in 1a (pass: {cp['C14']['passes'][0].split('**')[1]}); the pinion is quoted only after (1b line). Seller question 6 asks the same."),
        ("1b", "Bench supply (Mean Well SE-1000-24, 41.7 A), set 24.0 V",
         f"M1 arm {a_m1['cont']:.1f} A sustained, {a_m1['peak_rated']:.1f} A peak at the rated drive limit, {a_m1['peak']:.1f} A at the peak limit "
         "(electrical.md *Bench supply*)", "calculated; price sourced", "high (80 %)",
         "Overload is a **latched shutdown at 105–125 %** (43.8–52 A; SE-1000 spec), not current limiting: a peak over it is a category-0 drop and a re-power. "
         "Set 24.0 V, not 25.2 (Fable #2: the clamp's lowest turn-off is 25.44 V). 1a runs on the 10 A CC supply at 1 A."),
        ("1b", "Safety board (e-stop, contactor, pre-charge, one-shot discharge, clamp, watchdog; netlist v3: 100 V coil driver, AUX / K1 monitor, 12 V UVLO, branch trip)",
         f"netlist + block diagram + state table (electrical.md); heartbeat lost → no torque {st_m1['total'] * 1000:.1f} ms (SW2 50); "
         f"watchdog {wd:.0f} ms vs the loop's longest measured heartbeat gap {jmax:.1f} ms (sw/loop_jitter.py, this machine, loaded)",
         "netlist v3, simulated; **not drawn** (no KiCad in the tree), **logic race open** (Fable #2 M2: on a K1 drop-out the AUX edge triggers and "
         "clears U6; FORCE at +30 ms vs T7's 4 ms; no AUX debounce); not built", "medium (55 %)",
         "1b gate 2: KiCad schematic, AUX debounce, LATCH cleared directly, the coil-open case in the logic model, a named reviewer. Fable #2 also "
         "suggests a bought safety relay (as the base's G9SA) for e-stop / reset / welded contact, keeping the custom board for clamp, pre-charge, watchdog (electrical track)."),
        ("1b", "Step-1 rig (rig track: rnd/rig.py, docs/M1_RIG.md)", "not designed yet: today a 300 mm bar bolted to the bare output with no stops, no "
         "guard, and the software describes the yaw stage (can_map ratio 2.0, rated 10 / peak 22 N·m; calibrate seeks ±155° stops; the 2 kg soak is "
         "118 % of the bare motor's rated) (Fable #2 C1)", E, "low (40 %)",
         "1b gate 3: `step1` rig entry (ratio 1.0, 5 / 11 N·m, ±90°), steel stops at ≈ ±100° on a checked motor plate, bolted weights, shield or "
         "keep-out, soak 0.85 kg, a rehearsal of this rig. Its parts replace the 1b placeholder line."),
        ("1a / 1b", "Thermal (M7)", f"closed ASA sleeve: {th.get('dt_rated_sleeve', nan):.0f} K at rated → 0.65 × rated continuous; task-day winding "
         f"{th.get('task_max_c', nan):.0f} °C (old open + 10 K model {th.get('task_max_c_open_model', nan):.0f} °C); with a finned aluminium cap "
         f"{th.get('task_max_c_al_cap', nan):.0f} °C (thermal.md)", E, "low (45 %)",
         "Thermistors in 1a, read with the DMM by hand (no ADC in the kit); the case rise at rated torque is logged on the 1b rig; the aluminium cap is "
         "a Step-2 line after seller question 6 (which face carries the housing holes)."),
        ("1b", "Step-1b gate test kit (motorless tests T0–T16)", f"dummy drive bank, load resistors, 2 regen / bank Schottkys, the 0.33 Ω bank resistor, energy bank ${t['s1_test'] - t['s1b_instr']:,.0f}; "
         f"instruments ${t['s1b_instr']:,.0f} (scope with ×10 probes, two current clamps, 20 A CC supply, DMM; skip what the owner has; no differential probe, Fable #2)",
         "prices: list snapshots 2026-11-28 / " + E, "high (80 %)",
         "W15b: two 40 A Schottkys (VS-40CPQ060-N3: the regen source and the bank switch) and a 0.33 Ω 50 W bank resistor make T12's 30 A step "
         "(the 20 A supply + the 22 mF bank at 32.5 V); the CP2100A clamp's 800 kHz meets the sheet's ≥ 500 kHz"),
        ("2", "Arm structure (C1 / C3 / M5 CAD fixes: landed)", "C1 tie-rods with steel crush sleeves at the pitch hub and yaw flange (W8c: life preload 88 / 87 %), "
         "C45+QT clamp rings and a 7075-T6 yaw flange (Astra #3 m9: contact faces 1.34–2.93×), 4-ear cartridge flanges with nuts (W8b: tipping ≥ 10× at 30 % preload), "
         "rear cap v2 with dog ears (C3), D26 centres 81.48 / 71.21 with tension rings (Astra #3 m6: preload 134–220 N) (rnd/joints.py, belts.md)", C + "; creep data " + E, "medium (60 %)",
         "Open (Fable #2): joints.md lists the upper bone ↔ yaw flange screws, the detach clamp, the elbow core / clamp and the wrist output bearing as not "
         "drawn in the same wrench chain (M5); the R14 (c) driver probe fails 5 of 6 joints on its own Ø12 × 50 rule while a plain 3 mm hex key reaches "
         "the ears (C3: settle rule vs design); C13 measures the flange's μ and creep."),
        ("2", "Hold B: inline GIM8108-36 roll (the plan)", f"dome {ri.get('inline', {}).get('dome', {}).get('width_x', '?')} × "
         f"{ri.get('inline', {}).get('dome', {}).get('height_z', '?')} mm on the seller drawing (Ø96 × 57); boss dowels bonded 1.03× stall (cast-alloy "
         "boss assumed); W16c: the hard stop is two 7075 dogs on the adapter against a 7075 ring under the front collar (3.91× at stall)",
         C + "; output bearing (needs ≥ 1,417 N in service vs a 1,000 N listing), backlash, SDC303 protocol " + E, "medium (55 %)",
         "Seller questions 3–5. A different driver from Step 1's: protocol discovery again on arrival. Backlash 15 arcmin (ESTIMATE) → "
         f"hand {acc_raw:.1f} mm without joint encoders."),
        ("2", "Hold A: GIM8108-8 at pitch + elbow (D20)", (f"20 lb hug {hug:.2f}× rated (slim_study V1); " if hug else "") +
         "cartridge_m.parts_list_direct(MOTOR_8108): a C45+QT integral pinion shaft (v5: fatigue 0.68 → 2.63×; 8 × M3 × 6 10.9, 1.7×), a 6001 in wall B (2.0×), PU steel-cord "
         "15 mm belts, an 18 N·m flash limit (D26; kit lines ESTIMATE)", V + " / motor data " + E, "low (45 %)",
         "No datasheet; listings are 48 V nominal; the D20 drive is a redesign. **Do not buy until the 24 V answer.**"),
        ("2", "Hand accuracy and encoders (M3)", f"{acc_enc:.1f} mm with joint encoders, {acc_raw:.1f} mm without (accuracy.md, inline backlash in)",
         E, "low (40 %)", "The AS5048A mounts are drawn (rnd/parts/encoder.py, all four joints); the **RP2040 node firmware does not exist** (sw/): "
         "write it first, or drop the ≤ 1 mm claim for M1. Until then the wrist roll and gripper move through the USB-TTL adapter from the Pi."),
        ("2", "Printed structure (hand sag, R2b)", f"{st.get('total_mm', nan):.1f} mm raw, {st.get('after_mm', nan):.1f} mm after a gravity model "
         "(stiffness.json; P2 FEA chain; target ≤ 6 / ≤ 2)", "verified (FEA)", "medium (60 %)", "The bench measures it."),
        ("2", "Option: R17 roll latch (PIP_ROLL_GUARD=latch, default off)", f"O2 spring latch at +5°, {sd(t['opt'])} per arm ESTIMATE, ≈ 45 g (power_off.md); "
         "CAD + supervisor half in the tree", E, "medium (55 %)",
         "Owner decision (D27: nothing for now); not in any total. The print behind the guide sleeve carries ≈ 0.4 kN until a metal path to the motor face is drawn."),
        ("1a / 1b", "Bring-up software (sw/bringup.py)", "virtual bench: every step passes with a placeholder GIM table (5/5 joints); with the real table "
         "only the DM-J4310 is online; the Step-1 rehearsal models the yaw stage in the arm fixture, not the bare-motor lever (Fable #2 C1)",
         "verified (virtual bench)", "medium (55 %)", "The real GIM table is filled in 1a; the `step1` rig entry and its rehearsal are 1b gate 3 (sw / rig tracks)."),
    ]
    L = ["# M1 bench arm: purchase readiness (generated by `python -m rnd.m1_readiness`)", "",
         f"**Kit ([M1_BENCH_ARM.md](M1_BENCH_ARM.md)), with {CONTINGENCY:.0%} contingency:** Step 1a **${t['s1a_c']:,.0f}** (buy now; coupon tools "
         f"${t['s1a_tools_c']:,.0f} of it); Step 1b **${t['s1b_c']:,.0f}** (after the gate; test instruments ${t['s1b_instr_c']:,.0f} of it: "
         f"${t['s1b_c'] - t['s1b_instr_c']:,.0f} if the owner has them; rig parts a $0 placeholder until the rig track reports them); "
         f"Step 2 **${t['s2_plan_c']:,.0f}** (today's plan: GIM6010 at pitch + elbow, inline roll); whole kit ${t['plan_c']:,.0f}. Hold A (D20 with its drive parts) {sd(t['d_a'])}; "
         f"belt-roll fallback {sd(-t['d_b'])}; roll latch option {sd(t['opt_c'])} per arm (not in a total).", "",
         "## Go / no-go per step", "",
         "### Step 0: free — **do now**", ""]
    L += [f"{i}. {x}" for i, x in enumerate(STEP0, 1)]
    L += ["", f"### Step 1a: GIM6010-8 + DM-J4310 + CAN + 10 A CC supply + stand + lever stock + coupon parts (${t['s1a_c']:,.0f}) — **GO**",
          "- It answers the one question everything else hangs on (the SDC104 protocol) on a bare, unloaded motor at a 1 A limit, which needs no safety "
          "board (Fable #2 verdict). Every line except the supply, the stand, the lever stock and the coupon tools is reused in the arm.",
          "- **No weight on the lever and no lever on the motor beyond the C14 tilt loads** (1–4 kg hung at r 250, motor unpowered) until the 1b rig exists (Fable #2 C1).",
          "- The 1 A limit and the supply's output switch are the safety for 1a: discovery's `--enable` sends only zero-gain frames; the impedance "
          "check uses low gains at that limit.", "",
          "**Step 1a exit = the gate to buy Step 1b (all four):**"]
    L += [f"{i}. {x}" for i, x in enumerate(STEP1B_GATE, 1)]
    L += ["", f"### Step 1b: SE-1000-24 + safety board + test kit + catch + rig parts + pinion (${t['s1b_c']:,.0f}) — **NO-GO until the gate**",
          "- The board parts are ordered after the KiCad schematic, the logic-race fix and the signed review; the board passes T0–T16 with no motor "
          "(docs/rnd/safety_board_tests.md) before any drive is powered from it.",
          "- Astra review #3 (critical 3) simulation stays the basis: as drawn the clamp rang at MHz (filter + 50 µs minimum on-time; on 26.0 / off "
          "25.7 V), the coil driver was a 30 V part seeing ≈ 52 V, a stuck AUX bypassed pre-charge, a welded K1 / broken AUX went undetected, a dead "
          "12 V left K1 closed with no clamp, and the 20 A fuse left the in-arm 16 AWG unprotected at 18–35 A (netlist v3 fixes all six).", "",
          "**Step 1b exit criteria (all needed before Step 2 is bought):**",
          "1. **Gate before any drive is powered from the board:** the independent review checklist (docs/rnd/safety_board_tests.md) signed by a named "
          "reviewer who did not draw the board, then the board built and **every motorless test T0–T16 passed** (bench supply + dummy capacitor bank "
          "+ resistor bank + CC regen source, scope at the board, at the drive end and on the drains with ×10 probes and ground springs; pass/fail numbers "
          "from rnd/safety_board_sim.py): ARM refused with a mushroom pressed, without a heartbeat or with AUX stuck; pre-charge then K1; the 10 Ω dummy "
          "load trips the pre-charge timeout; FORCE on every opening, including a K1 coil opened by unplugging (the logic race fixed, T7), never during "
          f"pre-charge; WD_OK low {wd * 0.9:.1f}–{wd * 1.2:.0f} ms after the last edge and last edge → drive-end bus < 10 V ≤ 50 ms (scope, not software "
          "timestamps); no self re-arm; disarmed after a supply cycle; clamp on at 26.0 V ± 1 %, no MHz bursts, drain ≤ 48 V, drive end ≤ 27.5 V at the regen limit; "
          "loss of the 12 V, broken / stuck AUX and a linked-out K1 each end in a stop.",
          "2. The GIM table in sw/drivers/gim.py is filled from 1a (`python -m sw.drivers.gim`: no TODO).",
          "3. `python -m sw.bringup --joints yaw,wpitch` **with the `step1` rig entry** (ratio 1.0, ±90°) on the designed rig passes enumerate, limits "
          "(torque limit in flash, read back, kept over a power cycle), safety (e-stop → no torque ≤ 50 ms measured, watchdog, the drive's own CAN "
          "timeout), calibrate (against the rig's ≈ ±100° stops), jog and gravity (the lever + weight model). First loaded enable with the bare bar, "
          "the supply at 2 A and the drive speed limit set (Fable #2 C1).",
          f"4. The real loop's longest heartbeat gap (bring-up report) is under half the {wd:.0f} ms watchdog, **on the Pi, with the cores loaded as on the robot** "
          "(a UVC camera streaming + `stress-ng` at ≈ 75 % CPU; Neon #2 assumption 5). If it is not: run the loop SCHED_FIFO on an isolated core, or move the "
          f"heartbeat + supervisor out of Python (sw track); raising `watchdog_s` is allowed only up to {el_tmax:.0f} ms (the SW2 budget, electrical.md).",
          "5. Torque constant and friction measured on the rig's lever; the motor case temperature logged for 30 min at rated torque.",
          f"6. **Coupons C12–C14 pass** ([COUPONS.md](COUPONS.md), pass numbers from the repo's load cases): C12 insert pull-out ({cp['C12']['passes'][0].split(';')[0].split(' (the')[0]}; "
          f"{cp['C12']['passes'][1].split(' (2 ×')[0]}); C13 the bolted 4-ear flange (no slip at 16.5 N·m fresh and 13 N·m after 72 h at 50 °C, no "
          f"tipping gap at 16.1 N·m); C14 the drive stub (the output's tilt stiffness read in 1a; the 8 × M3 × 6 + Loctite 243 joint holds 16.5 N·m "
          "with the 360 N side load on the dummy output, and the drive's 11 N·m peak on the real output on the rig).",
          "7. **Soak test** (determinism; Asimov sim2real lesson 1, hour 20 = hour 1): `python -m sw.soak --joints yaw,wpitch --hours 2` passes on the rig "
          "with **0.85 kg at 0.3 m** (≈ 50 % of the bare GIM6010's 5 N·m rated; the 2 kg in sw/soak.py is the yaw-stage figure, Fable #2 C1). That is a "
          "sysid fit (friction, torque constant, backlash, latency) before and after 2 h of cyclic motion, with drift ≤ 20 % Coulomb, ≤ 25 % viscous, ≤ 5 % "
          "torque constant, ≤ +0.1° backlash, ≤ +0.5 ms mean / +2 ms p95 latency; no supervisor stop, no bus-off, no new throttle flag, zero corrupted CAN "
          "frames, loop jitter p99 < 1 ms on the Pi (PREEMPT_RT reported). **Open (Fable #2 minor, lead to decide before 1b):** whether jitter p99 < 1 ms "
          "in Python over USB-CAN blocks Step 2 (proposal: report it; criterion 4 is the blocking timing number).", "",
          "### Step 2: the rest of the arm — **NO-GO today**; GO when",
          "- Step 1b's exit criteria are met (if the SDC104 has no impedance mode: stop and redesign the GIM joints' safety argument first);",
          "- the seller has answered: the GIM8108-36's output face, bearing rating, backlash and the **SDC303 protocol** (same as the SDC104?); "
          "for hold A, the GIM8108-8's 24 V figures;",
          "- the arm's load chain is drawn (Fable #2 M5): every joint between the shoulder and the hand has a row in joints.md (upper bone ↔ yaw "
          "flange screws, the detach clamp, the elbow core / clamp, the wrist output bearing), and the R14 (c) driver rule vs the 3 mm hex key is settled (C3);",
          "- for hold A: the seller's 24 V answer (the D20 parts list: C45+QT pinions with 10.9 screws, PU steel-cord belts; D26);",
          "- for the ≤ 1 mm accuracy claim: the RP2040 node firmware (the encoder mounts are drawn; M3), else M1 proves everything but accuracy.", "",
          "## What is verified, measured, or still an ESTIMATE", "",
          "| Step | What you're buying / relying on | Evidence (checks in this repo) | Status | Confidence | Note |", "|---|---|---|---|---|---|"]
    L += [f"| {s} | {a} | {b} | {x} | {c} | {d} |" for s, a, b, x, c, d in rows]
    L += ["", f"**Still ESTIMATE:** the GIM protocols (all three drivers); the GIM6010-8 output bearing and pattern (C9 / C14); GIM8108-8 24 V figures; "
          f"GIM8108-36 output bearing and backlash; the D20 drive parts; the thermal network constants; the safety board's timing until measured; "
          f"insert pull-out, flange friction and creep (C12 / C13); {n_est} ESTIMATE-priced Step-1 lines; the rig's parts (placeholder).", "",
          "## Risks", "",
          "| Risk | Effect if it bites | Mitigation | Blocks |", "|---|---|---|---|",
          "| SDC104 has no impedance mode / CAN timeout / flash limit | the supervisor's torque bound does not hold on the GIM joints | 1a finds out on one $90 motor "
          "before 1b is bought; safety redesign before 1b | 1b, Step 2 |",
          "| The lever rig as first specified (no stops, no guard, software for the yaw stage) | a free-spinning loaded bar (≈ 31 J a revolution at 5 N·m) | "
          "no weight on the motor in 1a; 1b gate 3 (stops, guard, `step1` entry, 0.85 kg soak, rehearsal) | 1b |",
          "| Safety-board logic race (K1 drop-out) and AUX bounce | a FORCE glitch into the latch clear; a pre-charge re-close; \"arms most of the time\" | "
          "1b gate 2: debounce AUX, clear LATCH directly, model the coil-open case, T7 consistent | 1b |",
          "| GIM6010-8 output bearing is moment-rigid | the D26 stub at 0.56× in fatigue after a swap offset | C14 in 1a; overhung pinion (STUB_SUPPORT_RULE) | the pinion quote |",
          "| SDC303 (inline roll) speaks a different protocol | a second driver table; possibly no impedance mode on the roll | seller question 5; discovery on "
          "arrival; belt-roll fallback | hold B |",
          "| Watchdog trips on loop jitter on the Pi (Python GC, USB IRQ bursts) | category-0 drops | 25 ms sized on a measurement on x86 only; re-measure on the loaded Pi "
          "(exit criterion 4); SCHED_FIFO / isolated core or a non-Python heartbeat if it fails | Step 1b exit |",
          "| Safety board mis-built (the Neon #2 review found 5 netlist faults on paper) | no stop, an avalanched clamp, a self re-arm | netlist v3, KiCad + LTspice + a named reviewer before ordering, "
          "board tests before any motor | Step 1b exit |",
          "| Motors run hot in the closed sleeves | derating to ≈ 0.65 × rated | aluminium finned caps (Step 2, after seller question 6), cover vents, the drive's thermal limit | no |",
          "| GIM8108-8 is 48 V-only or slow at 24 V; D20 drive needs a redesign | D20 fails; 20 lb hug not met | hold A; GIM6010-8 fallback | hold A |",
          f"| Robot BOM over budget: ${bom.get('total', 0):,.0f} landed (limit $5,500, owner 2026-11-27), ${bom.get('with_d20', 0):,.0f} with D20, "
          f"base OFF-delay timer `PIP_BASE_TIMER={bsel}` (pending owner decision); with `{balt}` instead "
          f"${bt.get(balt, {}).get('total', 0):,.0f} / ${bt.get(balt, {}).get('with_d20', 0):,.0f} | the full robot needs a "
          "cost decision | the M1 parts are reused; ranked savings in docs/rnd/cost_down.md (the gate-neutral ones do not reach $5,500 alone); the decision is the owner's | no |",
          "| Category-0 drop on the bench | a falling arm | catch cradle ≤ 50 mm under the arm, tether, remote mushroom (Step 1b) | no |",
          "", "## Day-1 plan", "",
          "**Step 0 (before buying):** read the GIM6010-8 manual; send the six seller questions (below).", "",
          "**Step 1a (on arrival):**",
          "1. **Check and measure, unpowered:** the 24 V SDC104 version and the DM-J4310 V1.1; bolt the motor to the stand; C9 disc on both faces (PCDs); "
          "C14 (a) tilt and bearing layout, pilot and thread depth (motor output up, 1–4 kg at r 250).",
          "2. **Protocol discovery on the bare GIM6010-8** (alone on can0, the output bare with the C9 / C14 fixtures off, the 10 A supply at "
          "24.0 V / 1 A): vendor tool over USB-C first (node ID, firmware, 1 Mbit/s, MIT mode if offered), then `python -m sw.gim_discover --bus "
          "socketcan:can0 --id <n>` and, if the probe answers, `--enable` (zero gains, turn the output by hand). Then a low-gain impedance hold, "
          "the CAN timeout (stop sending: the drive must disable) and the flash limit (set, power-cycle, read back). Fill sw/drivers/gim.py; record "
          "what is still unknown (docs/sw/gim_protocol_notes.md).",
          "3. **IDs and buses:** DM-J4310 ESC_ID 0x05 / MST_ID 0x15 on the same bus; the GIM6010-8 as the yaw (0x03 / 0x13, can_map.yaml); terminations: "
          "adapter jumper on, far end terminated; the CAN HAT against the two adapters (docs/sw/can_interface_note.md).",
          "4. **Coupons on the bench:** print and test C12 (insert pull-out), C13 (4-ear flange: proof, μ, tipping, then the 72 h hold) and C14 (b) "
          "(the 8 × M3 joint on the dummy output); record the results in COUPONS.md.",
          "5. **Gate review:** discovery result + C14 in; the board schematic and the rig design from their tracks → the owner decides on 1b.", "",
          "**Step 1b (after the gate):** build the board and run T0–T16 with no motor; set the catch; mount the rig (stops, guard); the SE-1000-24 at "
          "24.0 V; `python -m sw.bringup --bus socketcan:can0,can1 --estop gpiod:gpiochip0:17:27 --joints yaw,wpitch` with the `step1` rig entry, each step "
          f"gated on the one before: enumerate → limits → safety (e-stop ≤ 50 ms, watchdog {wd:.0f} ms, drive CAN timeout) → calibrate → jog → gravity; "
          "then the lever tests (torque constant, friction, a collision trip, 30 min at rated torque with the case logged), C14 (c) and the soak.", "",
          "Rehearse first: `python -m sw.gim_discover --bus virtual --enable` and `python -m sw.bringup --bus virtual` (sw/test_s2.py).", "",
          "## Seller questions (copy of M1_BENCH_ARM.md)", ""]
    L += [f"{i}. {q}" for i, q in enumerate(SELLER_QUESTIONS, 1)]
    (D / "M1_READINESS.md").write_text("\n".join(L) + "\n")
    print("\n".join(L[:12]))


if __name__ == "__main__":
    main()
