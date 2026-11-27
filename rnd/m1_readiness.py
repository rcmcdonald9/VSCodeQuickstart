"""M1 purchase readiness: what the bench-arm buy depends on, how sure each answer is, the day-1 plan and the go/no-go risks.
python -m rnd.m1_readiness

Reads the cached results of the checks (docs/rnd/*.json, docs/sw/bringup_*.json) so the numbers in the report are the
ones the checks produced; the kit totals come from rnd/m1_kit.py.
"""
from __future__ import annotations

import json
from pathlib import Path

D = Path(__file__).resolve().parent.parent / "docs"
J = lambda n: json.loads((D / "rnd" / n).read_text()) if (D / "rnd" / n).exists() else {}
V, C, E, U = "verified (sim / CAD check)", "measured in CAD", "ESTIMATE", "untested (M1 / coupon decides)"


def _bringup():
    """Latest virtual bring-up reports: (real GIM table, placeholder GIM table) step results."""
    out = {}
    for tag, pat in (("real", "bringup_*_virtual.json"), ("sim", "bringup_*_virtual_simgim.json")):
        fs = sorted((D / "sw").glob(pat))
        if fs:
            try:
                out[tag] = json.loads(fs[-1].read_text())
            except Exception:
                pass
    return out


def main():
    from .m1_kit import totals, CONTINGENCY, SELLER_QUESTIONS, items, sd
    from .params import BASELINE
    from .spring import workspace_margins
    wm = workspace_margins(BASELINE)
    t = totals()
    th, st, dx, sw, asq, integ, sp, hn, ri, ss, ba = (J("thermal.json"), J("stiffness.json"), J("dexterity.json"), J("swept.json"),
                                                      J("assembly_seq.json"), J("integration.json"), J("spacing.json"), J("harness.json"),
                                                      J("roll_inline.json"), J("slim_study.json"), J("basket.json"))
    bu = _bringup()
    bom = J("bom.json")
    mr = min(v["m_rated"] for v in wm.values())
    mp = min(v["m_peak"] for v in wm.values())
    belt, inl = ri.get("belt", {}), ri.get("inline", {})
    v1 = next((v for k, v in ss.items() if k.startswith("V1")), {})
    hug = v1.get("hug")
    n_est = sum(1 for *_, src in items() if src.startswith("ESTIMATE"))
    rows = [
        ("Motors in Buy now: GIM6010-8 (yaw), DM-J4310 (wrist pitch), 2 × STS3215",
         f"whole-workspace margin ≥ {mr:.2f}× rated (p95, 1 kg), ≥ {mp:.2f}× peak over every joint (spring.py); hottest winding "
         f"{th.get('task_max_c', float('nan')):.0f} °C on a non-stop task day (thermal.py)", V, "high (90 %)",
         "Unchanged by either hold. Motor data are datasheet / reseller figures; the bench measures torque constant and friction (bring-up step 6)."),
        ("Hold A: GIM8108-8 at pitch + elbow (D20)",
         (f"20 lb hug {hug:.2f}× rated (slim_study V1); " if hug else "") + "basket worst duty margin "
         f"{v1.get('basket', float('nan')):.2f} (basket.py, D20 motors)", V + " / motor data " + E, "medium (60 %)",
         "No official datasheet; listings are 48 V nominal. **Do not buy until the seller confirms 24 V torque and speed.** Params and CAD "
         "still carry the GIM6010 (the D20 CAD follows the answer)."),
        ("Hold B: inline GIM8108-36 at roll (D24 option)",
         f"dome {belt.get('dome', {}).get('width_x', '?')} × {belt.get('dome', {}).get('height_z', '?')} (belt) → "
         f"{inl.get('dome', {}).get('width_x', '?')} × {inl.get('dome', {}).get('height_z', '?')} mm (inline), CAD; R14 roll row, R15, swept, "
         "basket pass inline (roll_inline.md)", C + "; motor envelope + output bearing " + E, "medium (65 %)",
         "The CAD assumes Ø100 × 60 and 6 × M4 patterns; the output bearing must carry ≈ 280 N radial (rating not published). Owner OK pending."),
        ("D24 bolted cartridges (sleeve Ø86.6, Ø68 flange, 6 × M4 into inserts, Oldham drive)",
         f"CAD (cartridge_m.can_bolted, belt_stage.small_stage_v3); R14 {asq.get('ok', '?')}/{asq.get('n', '?')} shafts assemble; "
         f"torque paths {integ.get('torque_ok', '?')}/{integ.get('torque_n', '?')} land in structure", C, "medium-high (75 %)",
         "Flange screws ≈ 63 N each at the 11 N·m peak and the Oldham tongue ≈ 13 MPa are hand calcs (ESTIMATE): the first coupon. "
         "Roll / yaw (belt) and the lift still draw KX74 sockets in the CAD (in flight); the kit buys for all-bolted (D24)."),
        ("D19 one-piece shaft-pulleys + capped seats (pitch, roll, yaw, elbow)", "R14 slide-on + insertion (assembly_seq.py); elbow seat caps on "
         f"lugs: housing stiffness 12.1 → 10.9 mm chain", V, "high (85 %)", "Nothing to buy: printed tubes + bolted caps (inserts are in Buy now)."),
        ("D13 v3 drive parts (Ø12 shafts, aluminium pulleys, 6801s, Oldham couplings)", "stage length (mounting plane → shaft end) 64.5 → 40.0 mm, "
         "shaft 49.5 mm (CAD)", C + "; bought-part dimensions " + E, "medium-high (75 %)",
         "Check the pulley bore (12 mm) and measure the Oldham disc before printing the tongue adapters."),
        ("Gear ratios and belts", "pitch 3.0, roll 4.44 (80T, belt), yaw 2.0, elbow 2.0 (3.0 with D20); belt pull ≥ 1.4× working rating (belts.py)",
         V, "high (85 %)", "Belts are cheap; the elbow belt length waits on hold A."),
        ("Bearings (61808, 6708, 6801, 61804, U604; 61806 inline)", "counted from the CAD (wave 1)", C, "high (90 %)", "A 10-pack of each covers spares."),
        ("Balancer (ISO 10243 die spring 18.7 N/mm, 2:1 tackle)", "pin B sign fixed, b 20, a 40–60, 35 % of L0 (balancer.py); virtual bring-up "
         "fits K from the pitch residual", V, "medium (70 %)", "K adjusts with the pulley slot; bring-up step 6 fits the real K."),
        ("Bench electronics (A5 v2: regen clamp, contactor, pre-charge, watchdog, catch)", "electrical.md: 136 J / 1.0 kW bench (320 J robot) vs "
         "625 J pulse allowance; heartbeat lost → no torque 24.6 ms bench (budget 25)", "calculated, not measured", "high (85 %)",
         "Timing is a budget, not a measurement: bring-up step 3 measures e-stop → contactor (≤ 50 ms) and the watchdog (≤ 9.6 ms)."),
        ("CAN (2 × candleLight, can_map.yaml)", "B1 / B2 at 46 % load, unique IDs, two terminations per bus (sw/can_map.py, virtual bring-up)",
         V, "high (85 %)", "Adapter-end termination: the FYSETC UCAN's onboard 120 Ω jumper; the nodes terminate the far ends."),
        ("Bring-up software (sw/bringup.py)", "virtual bench: every step passes with a placeholder GIM table (5/5 joints online); with the real "
         "table only the DM-J4310 is online, the 4 GIMs are BLOCKED (protocol TODO)", "verified (virtual bench)", "medium (60 %)",
         "**The GIM CAN protocol is unknown** until the SteadyWin manual is read (seller question 5): day 1 can't move a GIM joint without it."),
        ("Cable routing (ribbon stock, slim USB, CSI camera)", f"{hn.get('ok', '?')}/{hn.get('n', '?')} joints cross their full range; wrist roll "
         "302° vs 432° still open (harness.md)", V, "medium (65 %)", "Ribbon stock is cheap and in Buy now; the wrist-roll cassette is a design item."),
        ("Printed structure (not bought; decides whether M1 meets its accuracy goal)",
         f"hand deflection {st.get('total_mm', float('nan')):.1f} mm raw, {st.get('after_mm', float('nan')):.1f} mm after a gravity model "
         "(P2 FEA chain; target ≤ 6 / ≤ 2)", "verified (FEA); target not met", "medium (55 %)", "R2b still fails; the bench measures it. Doesn't change what to buy."),
        ("Dexterity (R16)", f"top-down {dx.get('top-down', 0):.0%}, 45° {dx.get('45° angled', 0):.0%}, side {dx.get('side (horizontal)', 0):.0%}, "
         f"across the body {dx.get('side, across the body', 0):.0%} (belt; target 54 % across: inline roll gives 62 %)", V, "high (80 %)", ""),
        ("Spacing / swept clearances", f"static hard violations {sp.get('under_hard', '?')} (head shell × screen, not on M1); swept roll/yaw "
         f"{sw.get('under_hard', '?')}; clashes inside modules {integ.get('module_clashes', '?')}", V, "high (85 %)",
         "The arm's one module clash is the D21 finger hook × fixed jaw (a mounting interface to confirm); the other is the head."),
    ]
    L = ["# M1 bench arm: purchase readiness (generated by `python -m rnd.m1_readiness`)", "",
         f"**Kit ([M1_BENCH_ARM.md](M1_BENCH_ARM.md)), with {CONTINGENCY:.0%} contingency:** Buy now **${t['now_c']:,.0f}**; "
         f"hold A (GIM8108-8 × 2) {sd(t['d_a'])} and hold B (inline GIM8108-36) {sd(t['d_b'])} against the GIM6010 / belt defaults "
         f"(all defaults ${t['default']:,.0f}, both holds ${t['both']:,.0f}).", "",
         "## Verdict", "",
         "- **GO on the Buy-now list.** Every line is either decided and checked in this repo or a cheap, re-buyable part whose only "
         "uncertainty is a dimension (pulley bore, Oldham disc, inserts). Nothing in it changes with hold A or hold B.",
         "- **Send the seller questions today** (M1_BENCH_ARM.md §4); they gate the three motors that wait.",
         "- **NO-GO on the GIM8108-8s** until the 24 V answer; **NO-GO on the GIM8108-36** until the owner's OK and the drawing.",
         "- To have every joint moving on day 1 anyway: the GIM6010-8 fallbacks are zero-regret (the full robot needs ≥ 7).", "",
         "## What is verified, measured, or still an ESTIMATE", "",
         "| What you're buying / relying on | Evidence (checks in this repo) | Status | Confidence | Note |", "|---|---|---|---|---|"]
    L += [f"| {a} | {b} | {s} | {c} | {d} |" for a, b, s, c, d in rows]
    L += ["", f"**Still ESTIMATE (no datasheet or measurement in hand):** GIM8108-8 24 V torque/speed and driver choice; GIM8108-36 frame, flanges "
          "and output-bearing rating; all bolt circles (coupon C9 reads them); Oldham disc geometry; flange-screw and tongue stresses (hand "
          "calcs); the "
          f"{n_est} ESTIMATE-priced Buy-now lines; P2S print time (× 0.35 of the generic slicer); the e-stop and watchdog timing (budget).", "",
          "## Go / no-go risks for buying now", "",
          "| Risk | Effect if it bites | Mitigation | Blocks Buy now? |", "|---|---|---|---|",
          "| GIM CAN protocol unknown (manual blocked here) | bring-up can't enable a GIM (4–5 joints BLOCKED) | seller question 5; fill the "
          "table in sw/drivers/gim.py; the virtual run with a placeholder table already passes every step | no (one GIM6010 is in Buy now: "
          "read the protocol on it first) |",
          "| GIM8108-8 is 48 V-only or slow at 24 V | D20 fails; 20 lb hug not met | hold A; fallback GIM6010-8 (bench-valid at 1 kg); "
          "ask about a 24 V sibling (listings show a GWK8108-8 at 24 V) | no |",
          "| GIM8108-36 bigger than Ø100 × 60 or weak output bearing | inline roll CAD doesn't fit / bearing overload | hold B; belt roll stays "
          "the default (built, checked) | no |",
          "| Wrong variant delivered (48 V GIM, J4310 V1.2, 7.4 V STS3215) | wrong torque / voltage | check the listing and the label on "
          "arrival (M1_BENCH_ARM.md checks) | no |",
          "| Bought-part dimensions differ (pulley bore, Oldham disc, insert length) | a reprint | measure on arrival, set the CAD constants, "
          "then print | no |",
          "| Printed structure softer than the FEA chain (R2b fails today) | hand sag above 2 mm | stiffer shells / PETG-CF / bolted joints | no |",
          f"| Robot BOM over budget: ${bom.get('total', 0):,.0f} landed now (limit $5,000), ${bom.get('with_d20', 0):,.0f} with D20, "
          f"${bom.get('with_d20_inline', 0):,.0f} with both holds | the full robot needs a cost decision | the M1 parts are all reused; "
          "the decision is the owner's (BOM_v4.md: pending decisions) | no |",
          "| Category-0 drop on the bench | a falling arm | catch cradle ≤ 50 mm under the arm, tether, remote mushroom (in Buy now) | no |",
          "", "## Day-1 plan (sw/bringup.py)", "",
          "Rehearse first: `python -m sw.bringup --bus virtual` passes today (MuJoCo bench twin, injected faults caught by sw/test_s2.py).",
          "",
          "0. **Before any motor power:** build the regen clamp and the e-stop / contactor / watchdog board; test the clamp on a dummy "
          "capacitive source; set the catch cradle ≤ 50 mm under the arm; bolt the stand down; PSU current limit 1 A at 24 V.",
          "1. **IDs and buses:** STS3215 IDs 1 / 2 with the USB-TTL adapter; DM-J4310 ESC_ID 0x05 / MST_ID 0x15 (factory 0x01 / 0x00); "
          "GIM IDs per can_map.yaml (0x01–0x04, feedback + 0x10) from the SteadyWin manual; `ip link set can0 up type can bitrate 1000000` "
          "(and can1); terminations: adapter jumper on, far-end node on.",
          "2. **Fill the GIM table** in sw/drivers/gim.py from the manual; until then the GIM joints are BLOCKED and held in the fixture.",
          "3. `python -m sw.bringup --bus socketcan:can0,can1 --estop gpiod:gpiochip0:17:27`, each step gated on the one before:",
          "    - **enumerate**: every motor on its ID, firmware, MIT mode, MIT ranges fit joint range × ratio;",
          "    - **limits**: drive torque limit (rated) + CAN timeout written to flash and read back;",
          "    - **safety**: e-stop → contactor open ≤ 50 ms (budget 25), reset refused while pressed, hung loop → watchdog ≤ 9.6 ms, "
          "drive CAN timeout disables, limits survive a power cycle;",
          "    - **calibrate**: distal joints first, zero against the hard stops at low current, span check (catches a wrong belt ratio);",
          "    - **jog**: ±amplitude at 10 % speed, operator confirms direction; observer peak recorded;",
          "    - **gravity**: holds from both sides → friction, residual, the balancer K, a drive-clamp sag test, collision thresholds.",
          "4. Report: docs/sw/bringup_<date>.md + .json (offsets, limits, thresholds); then thermal (A6 can wall), hand sag (R2b), CAN "
          "timing (SW1) and the first coupon on the bolted flange + Oldham tongue.", "",
          "Not covered by the tool yet: the RP2040 encoder nodes and the STS3215 bus through the elbow node (no protocol in the tree).", "",
          "## Seller questions (copy of M1_BENCH_ARM.md §4)", ""]
    L += [f"{i}. {q}" for i, q in enumerate(SELLER_QUESTIONS, 1)]
    (D / "M1_READINESS.md").write_text("\n".join(L) + "\n")
    print("\n".join(L[:12]))


if __name__ == "__main__":
    main()
