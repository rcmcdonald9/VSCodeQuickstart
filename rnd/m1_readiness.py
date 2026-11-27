"""M1 purchase readiness: what each step of the bench-arm buy depends on, how sure each answer is, the go / no-go per
step, the day-1 plan and the risks.  python -m rnd.m1_readiness

Reads the cached results of the checks (docs/rnd/*.json, docs/sw/*.json) so the numbers in the report are the ones the
checks produced; the kit totals come from rnd/m1_kit.py. Structure after the Fable review (2026-11-27): Step 1 (one GIM,
the protocol and the safety chain) and Step 2 (the rest of the arm), each with its own go / no-go.
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
    from .m1_kit import CONTINGENCY, SELLER_QUESTIONS, sd, step1, totals
    from .params import BASELINE
    from .spring import workspace_margins
    from . import accuracy, electrical as el
    wm = workspace_margins(BASELINE)
    t = totals()
    th, st, ri, ss, bom = J("thermal.json"), J("stiffness.json"), J("roll_inline.json"), J("slim_study.json"), J("bom.json")
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
    rows = [
        ("1", "Motors: one GIM6010-8, the DM-J4310, 2 × STS3215",
         f"workspace margin ≥ {mr:.2f}× rated at p95 (1 kg), **worst pose {worst[1]['rated'] / worst[1]['max']:.2f}× ({worst[0]})** (spring.py)",
         V, "high (85 %)", "Unchanged by either hold. The bench measures torque constant and friction on the test lever."),
        ("1", "GIM CAN protocol (SDC104)", "MIT mode named in the GIM6010-8 manual rev 1.2 and the GUI v2.0; no public CAN timeout, flash torque "
         "limit or status codes (docs/sw/gim_protocol_notes.md)", E, "low (40 %)",
         "**This is what Step 1 finds out** (`python -m sw.gim_discover`, rehearsed in sw/test_s2.py). If there is no impedance mode, the GIM joints "
         "need a different safety argument before Step 2."),
        ("1", "Bench supply (Mean Well SE-1000-24, 41.7 A)",
         f"M1 arm {a_m1['cont']:.1f} A sustained, {a_m1['peak_rated']:.1f} A peak at the rated drive limit, {a_m1['peak']:.1f} A at the peak limit "
         "(electrical.md *Bench supply*)", "calculated; price sourced", "high (80 %)",
         "Peak-limit strength tests one joint at a time (1.00× at all joints at once). The 10 A CC supply only does the 1 A first power-up."),
        ("1", "Safety board (e-stop, contactor, pre-charge, one-shot discharge, clamp, watchdog)",
         f"netlist + block diagram + state table (electrical.md); heartbeat lost → no torque {st_m1['total'] * 1000:.1f} ms (SW2 50); "
         f"watchdog {wd:.0f} ms vs the loop's longest measured heartbeat gap {jmax:.1f} ms (sw/loop_jitter.py, this machine, loaded)",
         "designed, measured jitter; not built", "medium (65 %)",
         "First-time build of a 146 A-pulse board: review the netlist, run the board tests (electrical.md) before any motor; re-run "
         "sw/loop_jitter.py on the Pi."),
        ("1", "Thermal (M7)", f"closed ASA sleeve: {th.get('dt_rated_sleeve', nan):.0f} K at rated → 0.65 × rated continuous; task-day winding "
         f"{th.get('task_max_c', nan):.0f} °C (old open + 10 K model {th.get('task_max_c_open_model', nan):.0f} °C); with a finned aluminium cap "
         f"{th.get('task_max_c_al_cap', nan):.0f} °C (thermal.md)", E, "low (45 %)",
         "Step 1 has thermistors and one aluminium cap to measure it on the real cartridge."),
        ("1", "Step-1 drive parts (one D13 stage: Ø12 shaft, 20T pulley, D40 Oldham, 315 belt, bearings)", "stage length 40.0 mm, shaft 49.5 mm (CAD)",
         C + "; bought-part dimensions " + E, "medium-high (75 %)", "Check the pulley bore (12 mm) and measure the Oldham disc before printing the tongue adapter."),
        ("2", "Arm structure (C1 / C3 / M5 CAD fixes)", "C1 tie-rods at the pitch hub and yaw flange + steel roll stops are on the C1/M1 track "
         "(rnd/joints.py); C3 rear cap / flange-screw access and M5 belt centres are open", U, "medium (55 %)",
         "**Do not print the pitch / yaw shafts or the cartridges as drawn** until those land; the Step-2 hardware lines follow them."),
        ("2", "Hold B: inline GIM8108-36 roll (the plan)", f"dome {ri.get('inline', {}).get('dome', {}).get('width_x', '?')} × "
         f"{ri.get('inline', {}).get('dome', {}).get('height_z', '?')} mm on the seller drawing (Ø96 × 57); boss dowels 1.6× stall; steel stops",
         C + "; output bearing, backlash, SDC303 protocol " + E, "medium (55 %)",
         "Seller questions 3–5. A different driver from Step 1's: protocol discovery again on arrival. Backlash 15 arcmin (ESTIMATE) → "
         f"hand {acc_raw:.1f} mm without joint encoders."),
        ("2", "Hold A: GIM8108-8 at pitch + elbow (D20)", (f"20 lb hug {hug:.2f}× rated (slim_study V1); " if hug else "") +
         "needs a Ø15 keyed stage, Ø50 Oldhams, 25 mm belts and 61902s for 22 N·m (Fable M6; kit lines ESTIMATE)", V + " / motor data " + E, "low (45 %)",
         "No datasheet; listings are 48 V nominal; the D20 drive is a redesign. **Do not buy until the 24 V answer and the D20 parts list.**"),
        ("2", "Hand accuracy and encoders (M3)", f"{acc_enc:.1f} mm with joint encoders, {acc_raw:.1f} mm without (accuracy.md, inline backlash in)",
         E, "low (40 %)", "The AS5048A mounts and the RP2040 node firmware do not exist yet: build them first, or drop the ≤ 1 mm claim for M1."),
        ("2", "Printed structure (hand sag, R2b)", f"{st.get('total_mm', nan):.1f} mm raw, {st.get('after_mm', nan):.1f} mm after a gravity model "
         "(stiffness.json; P2 FEA chain; target ≤ 6 / ≤ 2)", "verified (FEA)", "medium (60 %)", "The bench measures it."),
        ("2", "Bring-up software (sw/bringup.py)", "virtual bench: every step passes with a placeholder GIM table (5/5 joints); with the real table "
         "only the DM-J4310 is online", "verified (virtual bench)", "medium (60 %)", "The real GIM table is filled from Step 1."),
    ]
    L = ["# M1 bench arm: purchase readiness (generated by `python -m rnd.m1_readiness`)", "",
         f"**Kit ([M1_BENCH_ARM.md](M1_BENCH_ARM.md)), with {CONTINGENCY:.0%} contingency:** Step 1 **${t['s1_c']:,.0f}**; Step 2 **${t['s2_plan_c']:,.0f}** "
         f"(today's plan: GIM6010 at pitch + elbow, inline roll); whole kit ${t['plan_c']:,.0f}. Hold A (D20 with its drive parts) {sd(t['d_a'])}; "
         f"belt-roll fallback {sd(-t['d_b'])}.", "",
         "## Go / no-go per step", "",
         "### Step 1: one GIM6010-8 + DM-J4310 + servos + CAN + bench supply + safety board + one D13 stage — **GO**",
         "- Nothing in it waits on a seller answer or a CAD fix; every line is decided, and all but the two supplies, the stand, the test lever "
         "and the catch are reused in the arm.",
         "- It is the cheapest way to settle the biggest unknown: whether the SDC104 has a usable impedance mode, a CAN timeout and a torque "
         "limit in flash (Fable M4).",
         "- Send the seller questions the same day (below); they gate Step 2's motors.", "",
         "**Step 1 exit criteria (all needed before Step 2 is bought):**",
         "1. Safety board built and its bench tests passed without a motor: ARM refused with a mushroom pressed or without a heartbeat; "
         "pre-charge then K1; the 10 Ω dummy load trips the pre-charge timeout; the discharge one-shot fires on every opening and never during "
         f"pre-charge; heartbeat stopped → K1 open ≤ {wd * 1.2:.0f} ms; clamp at 26.5 V.",
         "2. `python -m sw.gim_discover` on the GIM6010-8: an MIT / impedance mode answers, and the GIM table in sw/drivers/gim.py is filled "
         "(`python -m sw.drivers.gim`: no TODO).",
         "3. `python -m sw.bringup --joints yaw,wpitch` on the Step-1 rig passes enumerate, limits (torque limit in flash, read back, kept over a "
         "power cycle), safety (e-stop → no torque ≤ 50 ms measured, watchdog, the drive's own CAN timeout), calibrate, jog and gravity.",
         f"4. The real loop's longest heartbeat gap (bring-up report) is under half the {wd:.0f} ms watchdog, on the Pi.",
         "5. Torque constant and friction measured on the lever; sleeve and aluminium-cap temperatures logged for 30 min at rated torque.",
         "6. Coupons: insert pull-out (C1) and the bolted flange + Oldham tongue.",
         "7. **Soak test** (determinism; Asimov sim2real lesson 1, hour 20 = hour 1): `python -m sw.soak --joints yaw,wpitch --hours 2` passes. That is a sysid fit (friction, torque constant, backlash, latency) before and after 2 h of cyclic motion at ≈ 50 % rated, with drift ≤ 20 % Coulomb, ≤ 25 % viscous, ≤ 5 % torque constant, ≤ +0.1° backlash, ≤ +0.5 ms mean / +2 ms p95 latency; no supervisor stop, no bus-off, no new throttle flag, zero corrupted CAN frames, loop jitter p99 < 1 ms on the Pi (PREEMPT_RT reported). The bring-up report's determinism section is clean (docs/sw/sim2real_bench.md).", "",
         "### Step 2: the rest of the arm — **NO-GO today**; GO when",
         "- Step 1's exit criteria are met (if the SDC104 has no impedance mode: stop and redesign the GIM joints' safety argument first);",
         "- the seller has answered: the GIM8108-36's output face, bearing rating, backlash and the **SDC303 protocol** (same as the SDC104?); "
         "for hold A, the GIM8108-8's 24 V figures;",
         "- the CAD fixes have landed: C1 (tie-rods, on the C1/M1 track), C3 (rear cap that can react torque, reachable flange screws), M1 "
         "(inline boss and steel stops, on the C1/M1 track), M5 (belt centres 80.9 / 70.7);",
         "- for hold A: the D20 drive redesign's parts list (Ø15 keyed stage, Ø50 Oldham, 25 mm belt);",
         "- for the ≤ 1 mm accuracy claim: encoder mounts + node firmware (M3), else M1 proves everything but accuracy.", "",
         "## What is verified, measured, or still an ESTIMATE", "",
         "| Step | What you're buying / relying on | Evidence (checks in this repo) | Status | Confidence | Note |", "|---|---|---|---|---|---|"]
    L += [f"| {s} | {a} | {b} | {x} | {c} | {d} |" for s, a, b, x, c, d in rows]
    L += ["", f"**Still ESTIMATE:** the GIM protocols (all three drivers); GIM8108-8 24 V figures; GIM8108-36 output bearing and backlash; the "
          f"D20 drive parts; the thermal network constants; the safety board's timing until measured; {n_est} ESTIMATE-priced Step-1 lines; "
          "bolt circles (coupon C9).", "",
          "## Risks", "",
          "| Risk | Effect if it bites | Mitigation | Blocks |", "|---|---|---|---|",
          "| SDC104 has no impedance mode / CAN timeout / flash limit | the supervisor's torque bound does not hold on the GIM joints | Step 1 finds out on one $90 motor; "
          "safety redesign before Step 2 | Step 2 |",
          "| SDC303 (inline roll) speaks a different protocol | a second driver table; possibly no impedance mode on the roll | seller question 5; discovery on "
          "arrival; belt-roll fallback | hold B |",
          "| Watchdog trips on loop jitter on the Pi | category-0 drops | 25 ms sized on a measurement here; re-measure on the Pi (exit criterion 4) | Step 1 exit |",
          "| Safety board mis-built | no stop, or a pre-charge that never completes | netlist + board tests before any motor | Step 1 exit |",
          "| Motors run hot in the closed sleeves | derating to ≈ 0.65 × rated | aluminium finned caps (in the kit), cover vents, the drive's thermal limit | no |",
          "| GIM8108-8 is 48 V-only or slow at 24 V; D20 drive needs a redesign | D20 fails; 20 lb hug not met | hold A; GIM6010-8 fallback | hold A |",
          f"| Robot BOM over budget: ${bom.get('total', 0):,.0f} landed (limit $5,500, owner 2026-11-27), ${bom.get('with_d20', 0):,.0f} with D20 | the full robot needs a "
          "cost decision | the M1 parts are reused; the decision is the owner's (BOM_v4.md) | no |",
          "| Category-0 drop on the bench | a falling arm | catch cradle ≤ 50 mm under the arm, tether, remote mushroom (Step 1) | no |",
          "", "## Day-1 plan (Step 1)", "",
          "0. **Before any motor power:** build the safety board (electrical.md netlist) and run its bench tests; set the catch cradle; bolt the "
          "stand down; the 10 A supply limited to 1 A for the first power-up, then the SE-1000-24.",
          "1. **Protocol discovery on the one GIM6010-8** (alone on can0, unloaded, in its fixture): vendor tool over USB-C first (node ID, "
          "firmware, 1 Mbit/s, MIT mode if offered), then `python -m sw.gim_discover --bus socketcan:can0 --id <n>` and, if the probe answers, "
          "`--enable` (zero gains, turn the output by hand). Fill sw/drivers/gim.py; record what is still unknown (docs/sw/gim_protocol_notes.md).",
          "2. **IDs and buses:** STS3215 IDs 1 / 2 with the USB-TTL adapter; DM-J4310 ESC_ID 0x05 / MST_ID 0x15; the GIM6010-8 as the yaw "
          "(0x03 / 0x13, can_map.yaml); terminations: adapter jumper on, far end terminated.",
          "3. `python -m sw.bringup --bus socketcan:can0,can1 --estop gpiod:gpiochip0:17:27 --joints yaw,wpitch` on the Step-1 rig (the GIM "
          "stage on its lever), each step gated on the one before: enumerate → limits → safety (e-stop ≤ 50 ms, watchdog "
          f"{wd:.0f} ms, drive CAN timeout) → calibrate → jog → gravity.",
          "4. Lever tests: torque constant, friction, the drive-clamp sag, a collision trip; 30 min at rated torque with the sleeve and the "
          "aluminium cap logged; the coupons.", "",
          "Rehearse first: `python -m sw.gim_discover --bus virtual --enable` and `python -m sw.bringup --bus virtual` (sw/test_s2.py).", "",
          "## Seller questions (copy of M1_BENCH_ARM.md)", ""]
    L += [f"{i}. {q}" for i, q in enumerate(SELLER_QUESTIONS, 1)]
    (D / "M1_READINESS.md").write_text("\n".join(L) + "\n")
    print("\n".join(L[:20]))


if __name__ == "__main__":
    main()
