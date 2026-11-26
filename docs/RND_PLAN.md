# Pip v4: hardware R&D, operating model and plan

**Scope:** the robot's **physical design** (mechanical, electrical, industrial design, simulation, cost). The AI stack comes later.

**Concept:** [V4_CONCEPT.md](V4_CONCEPT.md). **Live numbers:** [rnd/SCORECARD.md](rnd/SCORECARD.md). **History:** [RND_LOG.md](RND_LOG.md).

**Owner's brief (2026-11-25):** Claude runs the development. Iterate over long periods, innovate, change course when the evidence says so, and don't stop at the first "done". More compute → a better robot.

---

## 1. Operating model: a continuous engineering loop

Each scheduled run (and any interactive session) does one **loop**:

1. **Sync:** `git pull`; read the scorecard, the work queue (§4), and the last few log entries.
2. **Critique.** Ask what the single highest-leverage thing is right now. Candidates:
   - the **weakest scorecard metric**
   - the **biggest unretired risk**
   - the **next thing a build needs**
   - a **better idea** that could beat the current design (look for precedent first)
3. **Do the work** with real tools:
   - build123d CAD
   - MuJoCo twin and analyses (`python -m rnd.run_all`)
   - Blender renders
   - web research with sources
   - parallel subagents for independent research, where useful
4. **Measure:** regenerate the reports and update the scorecard. **A change that makes the scorecard worse is reverted unless the log explains why it's worth it.**
5. **Decide:** update the decision table (§3). Decisions **can be reversed**; record why. Edit the queue freely: add, split, reorder or delete.
6. **Record:** commit and push to `claude/robot-dog-p2s-printer-j4esie`, and add a log entry: what, numbers before → after, files, open questions.
7. **Tell the owner:** a short summary every run. A **digest** (renders + scorecard delta + decisions taken) every ~4 runs, or after any major change. Digests **don't block**; work continues.

**External review budget (owner, 2026-11-26):** Gemini 4 Neon (`rnd/external_review.py`) costs ≈ $1.67 per run. Use it **at most once per day**, and only after a significant batch of fixes. The script refuses a second run within 24 h.

**Cadence (owner, 2026-11-26):** runs every 30 minutes (two hourly scheduled tasks at :10 and :40, since one task can't run more often than hourly).

**Two tracks (owner, 2026-11-26):** hardware (this plan) and **software** ([SOFTWARE_PLAN.md](SOFTWARE_PLAN.md), starting with the bench arm). Runs **alternate**: the :10 run does hardware, the :40 run does software, unless one track is blocked.

**Efficiency rules (owner, 2026-11-26: "make more progress per run"):**
1. **One concrete deliverable per run**, picked in under 2 minutes from the queue top. No re-deliberating settled decisions.
2. **Incremental checks only.** Run the checks that cover what changed:
   - CAD of the arm → integration, spacing, assembly_seq
   - masses → spacing/workspace via `run_all --quick`

   The slow full set (FEA, dynamic sim, print plan, Monte Carlos, full `run_all`) runs only when its inputs changed, and at most every 4th run.
3. **Long jobs go to the background** while the next step proceeds. Run independent checks in parallel processes.
4. **Commit early** (at least once per run), with a short log entry (5 lines max) and a short owner summary (3 lines max).
5. **Don't chase the last 1 %:** stop when the deliverable passes its check. Queue polish separately.

**The loop never "finishes".** When the queue is empty, the next run starts with a fresh critique of the whole design and writes a new queue: a stronger design, cheaper parts, easier assembly, better looks.

**Guardrails:**
- No purchases, PRs or AI-stack work.
- Every part ≤ 256 mm (P2S).
- Every actuator in a joint cartridge.
- Numbers come with sources or are marked `ESTIMATE`.
- Keep `rnd/params.py` the single source of truth.
- Only the owner decides to **buy hardware**. Milestones below end with a buy/no-buy question.

## 2. Requirements (measurable)

| ID | Requirement | Target | Hard limit |
|---|---|---|---|
| R1 | Reach | Floor to 1.40 m: floor, dishwasher, table 0.75 m, counter 0.90 m, shelves 1.2 and 1.4 m, out to 0.60 m from the robot centre | Floor to 1.25 m |
| R2 | Payload per arm | 1.0 kg at 45 cm; 2.5 kg two-handed near the body | 0.6 kg at reach |
| R3 | Carry up stairs | ≤ 2 trips, pieces ≤ 10 kg with handles, split/join ≤ 30 s without tools, "carry mode" | Pieces ≤ 13 kg |
| R4 | Carpet | Low and medium pile, rugs, 15 mm thresholds, accurate position estimate | Low pile + 10 mm |
| R5 | Footprint | Base ≤ 50 cm; fits 70 cm doors with arms folded | ≤ 60 cm |
| R6 | Speed | 0.8 m/s, smooth holonomic motion | 0.5 m/s |
| R7 | Runtime | 2 h light duty | 1 h |
| R8 | Cost (excluding Pi) | ≤ $4,000 (was $3,300; raised for the D9 sensor pack, owner 2026-11-25) | $5,000 |
| R9 | Reusable actuators | All motors in joint cartridges | – |
| R10 | Assembly | First build ≤ 4 h hands-on; reconfiguration ≤ 1 h | – |
| R11 | Looks and motion | Reads as a robot: clean shells, expressed joints, light rings, expressive head, smooth motion | – |
| R12 | Safety | Torque-limited joints, e-stop, protected 24 V, stable (tip margin ≥ 50 mm under worst static load) | – |
| R13 | Printability | Parts ≤ 256 mm, support-free where possible | – |
| R14 | **Assembly simulation** (owner, 2026-11-26) | Every part has a simulated assembly sequence: (a) each bearing slides onto its journal from a free end; (b) each part reaches its installed position along a straight insertion path without colliding with parts already installed; (c) every screw has tool access (Ø12 mm clear along its axis for ≥ 50 mm for a hex key or driver); (d) every joint cartridge can be swapped with the arm assembled. Checked by `rnd/assembly_seq.py` | 100 % of parts | all drive shafts and cartridges |
| R15 | **Spacing** (owner, 2026-11-26) | Minimum clearance between parts that are not fused or bolted together: ≥ 1.0 mm static; running clearances set by a bearing, shim or guide ≥ 0.3 mm (loop 36 refinement); ≥ 2.0 mm between other parts in relative motion over their full joint range (swept); belts ≥ 3 mm from any static part; cable bend radius ≥ 10 × OD; no belt/pulley nip reachable by a 12 mm finger probe. Checked by `rnd/spacing.py` | as stated | ≥ 0.5 mm static, ≥ 1.0 mm moving |

## 3. Decisions (living; reversals logged)

| # | Decision | Status | Evidence |
|---|---|---|---|
| D1 | **Carry split: upper core / base / battery / 2 arms.** Each arm detaches at the shoulder-pitch output with a **KX-V V-band clamp** (printed 3" sanitary flanges, 3 dowels, stock quick-release clamp; loop 7: the twist collar can't carry the arm's bending) plus a blind-mate plug. Core 6.9 kg, arm 3.4 kg each, base 5.8 kg, battery 1.5 kg. Two trips: core + base, then battery + arms. *Changed* (loop 4): CAD masses put the one-piece upper at 11.9 kg (limit 10) | Changed 2026-11-25 (loop 4) | [budget](rnd/budget.md) |
| D2 | **Base: 4 × 6" omni, one wheel pointing forward, wheels at R 0.21 m, Ø500 shell.** *Reversed from 3-wheel kiwi:* kiwi tip margin was ~35–46 mm vs 66–82 mm | Changed 2026-11-25 | [twin](rnd/twin.md) |
| D2b | Cross thresholds at a **45° heading and ≤ 0.15 m/s** (loop 12 roller-level sim: 0° fails at 15 mm; 0.30 m/s reaches the GIM peak by impact, 0.15 m/s peaks ~6 N·m, 1.8×). Cap base acceleration at 1.0 m/s² with arms parked and 0.5 m/s² while manipulating. Position from optical flow + IMU + lidar | Updated (loop 12) | [threshold](rnd/threshold.md) |
| D3 | **Joint cartridges carry torque only (KX v1).** The cartridge output has a KX male face: 3 hardened balls on 3 pairs of steel dowel pins, locked by a spring-preloaded breech-lock collar (35° twist, thumb tabs, self-locking 6° ramp). It locks into a **hub that turns in the bone's own bearing pair**, and the housing is held by a **steel torque pin in a radial slot**. Sizes: KX74 (GIM6010-8), **KX56 (DM-J4310; added loop 4, saves ~45 g per socket and shrinks the wrist collar from Ø98 to Ø78)** and KX44 (STS3215). The seat doubles as an overload clutch (slip ≥ 1.16× actuator peak). Buses: 24 V CAN 1 Mbps / 12 V TTL. *Changed* from the same printed male face on both sides of every cartridge: that put arm bending (up to 12 N·m) and belt pull through printed V-grooves at 3–4× PETG yield. | Changed 2026-11-25 (loop 2) | [kx](rnd/kx.md) |
| D4 | **Arm: 0.30 + 0.28 + 0.13 m, wrist ±120°** (loop 1: ±140° folded the hand into the forearm; ±120° gives more collision-free reach). **CAD requirement:** forearm bone ends ≥ 60 mm before the wrist axis. Shoulder pitch GIM × 2.5 belt + **adjustable balancer K = 4 N·m** (τ = K·sin pitch; zero-free-length cable + 2.7 N/mm caged compression spring, pulley A on a 45–65 mm slot → K 3.6–5.2 N·m, [balancer](rnd/balancer.md)), roll GIM × **3.5**, upper-arm yaw **GIM × 2 belt** (loop 5: DM-J4310 direct was under-rated over the workspace), elbow GIM × 2, wrist pitch **DM-J4310**, wrist roll and gripper STS3215. Joints are sized on a **whole-workspace check** ([spring](rnd/spring.md)), not only the straight-arm case. *Changed* from 0.25 + 0.23, ±110°, wrist STS (reach coverage 31 % → 100 %; STS wrist was under-rated) | Changed 2026-11-25 | [arm sizing](rnd/arm_sizing.md), [twin](rnd/twin.md) |
| D4b | **Counter grasps use top-down or 45° approach, whichever is collision-free** (top-down alone reaches 62 % of counter cells, combined 100 %). Telescoping spine deferred: it would cost ~0.6 kg on a tight carry budget. Revisit if real tests show the need. | New (loop 1b) | [twin](rnd/twin.md) |
| D4d | **Bones: 1.2 mm printed shells + carbon tubes** for the upper-arm neck (Ø40×1) and the forearm link (Ø30×1), bonded into printed sockets. Stiffness, not strength, limits printed bones: the hand sagged 4.1 mm (printed only) → **1.6 mm** with CF. Precedent: Hello Robot Stretch, Kinova carbon links | New (loop 9) | [bones](rnd/bones.md) |
| D5 | **Lift: shoulder height 0.35–1.05 m. Standard GIM6010-8 cartridge on a 20T HTD 5M belt along the spine, with a spring-applied, solenoid-released ratchet pawl** (fail-safe, 0 W holding, safe carry mode). 2.1× torque margin, 3.5 s full travel, 0.55 kg. *Changed* from a NEMA23 + brake (1.25 kg, another motor type); constant-force springs rejected (they shoot up when the arms are removed; fatigue) | Changed 2026-11-25 (loop 8) | [lift](rnd/lift.md) |
| D8 | **Head fixed on top of the mast** (owner's suggestion). The torso and arms ride the lift below it. Overall height is a constant **1.29 m**. The head camera always has the high vantage (tilts 60° down for floor work, level/up for shelves; precedent: Hello Robot Stretch). The bare mast no longer pokes through the head, and 0.6 kg leaves the lift's moving mass. Reach 98 %, bimanual 4/4 and tip margin unchanged | New 2026-11-25 | [head views](rnd/img/head_camera_views.png), [render](rnd/img/robot_head_on_mast.png) |
| D9 | **Sensors favour accuracy and dexterity over cost** (owner, 2026-11-25). Sensor pack v2: joint-side absolute encoders on the 8 arm belt joints + lift (hand error 4.0 → 0.9 mm); global-shutter wrist cameras; magnetic-elastomer fingertip tactile pads (AnySkin/ReSkin principle); palm ToF; base cliff ToF; optical flow kept; 3 RP2040 CAN sensor nodes. +$276 | New (loop 15) | [accuracy](rnd/accuracy.md), [BOM](rnd/BOM_v4.md) |
| D10 | **Compute:** the owner's Pi 5 runs I/O, real-time supervision, IK and state estimation (≈ 73 % of 4 cores, ESTIMATE). The torso **reserves a bay + 19 V 3 A branch + Ethernet for a Jetson-class AI computer**, bought in the AI phase (~$249) | New (loop 17) | [compute](rnd/compute.md) |
| D11 | **Head face screen + touch panel** (owner, 2026-11-26). A bar-format LCD (~7–8", to fit the 196 × 66 mm visor; panel dimensions to verify) shows the face (the eyes are drawn on it) and status, and gives touch controls: pause/resume, home/park, joint jog, teach mode, task pick, and a STOP command. It runs on **its own ESP32-S3-class node on the CAN bus**, not on the Pi, so it still works if the Pi hangs, and the cable to the fixed head is power + one CAN pair. **The hard-wired torso e-stop stays the safety device**; the screen's STOP is a command beside it, not instead of it. ~$50–90, ~+150 g at the mast top, ~2–3 W | Adopted; design in queue item 5 | owner Q&A 2026-11-26 |
| D12 | **Cable routing: a clock-spring spiral at every joint; slim bundle.** Torsion through the short shaft bores can't absorb the ranges (cable ~±360°/m). The wrist camera becomes MIPI-CSI on an FFC through the wrist, with a CSI→USB bridge on the forearm and a slim 4-core USB (≤ Ø2.2) onward. The stiffest cable is then Ø2.2 mm (bend r 22 mm). | Adopted (loop 32); yaw cavity open | [harness.md](rnd/harness.md) |
| D13 | **Belt stage v2 (assemblability, review #1 finding 1).** Small-pulley stage: a **steel Ø12 shaft** in 2 × 6801 (12×21×5) inside thicker eccentric bushings; an **off-the-shelf aluminium HTD 5M pulley** (20T; **18T on the roll stage**, fixing the 20/18 mismatch) with set screws on a flat; a **clamp flange coupling** bolted to the printed KX74 hub on a 4 × M4 circle (≈ 5 MPa on the PETG at 12.7 N·m). The bearings slide on from the free end; the shaft drops through the walls. Big-pulley stages (pitch 60T, roll 72T, yaw 40T, elbow 40T): the printed pulley becomes a **separate part bolted to a shaft flange** (≥ 4 × M4), inserted between the walls before the shaft. Every shaft→link joint gets a key or a bolted flange. ≈ +$68 net for 9 small stages (6801s replace 61804s). | Adopted (loop 35); CAD in queue A1 | [assembly_seq.md](rnd/assembly_seq.md), review #1 |
| D6 | Toolchain: build123d CAD, MuJoCo twin generated from `rnd/params.py`, **arm collision geometry from CAD hulls** (`rnd/parts/collision.py`, loop 6), Blender renders | Holding | `rnd/` |
| D4c | **Belt stages: HTD 5M, 15 mm, 20T small pulley** (pitch 20:50, roll 20:60, elbow 20:40), printed pulleys, the small pulley on its own bearing hub. **Elbow motor stays at the elbow:** a remote motor on a long belt (Blue-style) was evaluated and rejected (–0.5 N·m at the shoulder, +0.24 kg) | New (loop 3) | [belts](rnd/belts.md) |
| D7 | **Power bus 24 V (6S Li-ion)**, replacing 48 V | Holding | [T01](rnd/T01_specs.md) |

## 4. Work queue (ordered; edit freely)

**Milestone M2 reached (2026-11-25 build-out session):**
- every module in CAD; full assembly with 0 cross-module clashes; 59/59 printed pieces fit the P2S
- dynamic twin v1 completes a drive + floor pick + table place within real actuator limits (8/8 robust)
- electrical plan, itemised BOM ($3,220), print plan (10.4 kg, ~255 h P2S) and build guide (~25 h hands-on)
- see [BUILD_v4.md](BUILD_v4.md)

Next (fresh critique after loop 31: torque paths 21/21, module clashes 0, docking and face screen designed, every confidence Monte Carlo passing; **scorecard 38 ✅ / 1 ❌, R10 build time only**). The hourly loop works top-down:

**Software track queue** (see SOFTWARE_PLAN.md):
- S0.1 bench env
- S0.3 synthetic demos in LeRobot format
- S0.2 robot description export
- S0.4 pipeline dry run with a pretrained checkpoint + open-data slice
- (S1+ need the M1 kit)

**External review #1 (Gemini 4 Neon, 2026-11-26) → queue A-items first** ([triage](review/triage_2026-11-26.md)):
- **R14/R15 (owner, 2026-11-26), new design targets:** extend `rnd/assembly_seq.py` to straight-line insertion paths for every part, tool access for every screw and cartridge swap with the arm assembled. *`rnd/spacing.py` done in loop 36 (static; 7 → 2 hard violations, both the elbow cassette vs its pulley/belt → fixed by the D13 elbow redesign). Still to do: swept spacing over joint ranges, finger-probe nips, cable bend radius in CAD.*
- A1 assemblability: *check built, loop 35: 1/6 shafts pass; D13 adopted.* *Loop 37: small-stage v2 CAD done (`belt_stage.small_stage_v2`, `V2` stack: bearings at −24.7…−29.7 / −57.7…−62.7, belt centre −40.2, 18 mm longer than v1; passes R14 standalone).* *Loop 38: elbow migrated (cassette plates re-derived from V2, belt plane y 20.2, posts x ±35; **R15 hard violations 2 → 0**).* *Loop 39: shoulder pitch migrated (Y_F −58 → −78, walls from V2, Y_BAL −3).* Next: roll (18T), yaw, lift; then the big-pulley split shafts (elbow 40T hub first); BOM lines for D13
- A2 KX socket construction
- A3 real clock-spring cavities + harness
- A4 wrist bearings + head tilt
- A5 e-stop electronics
- A6 PETG thermal limit
- A7 roll 18T + small_hub FEA
- A8 FEA method + compliance chain
- A9 balancer redesign
- A10 tip cases
- A11 wheels without KX + chassis ring
- A12 dock fixes
- A13 BMS
- A14 CAN rates
- A15 lift lock
- A16 BOM contingency

1. ~~Yaw cable crossing~~ *(loop 33: **yaw ±150° → ±70°**, crossed in torsion over ≈ 0.24 m between clamps. Reach 99 % and bimanual 4/4 unchanged; ±60° loses 3 of 4 bimanual tasks; the 12 mm cassette option dropped the shoulder-roll margin to 1.49×.)* *(Loop 34: yaw hard stops done: a Ø6 pin in the bottom yaw plate rides an arc slot in the yaw-shaft flange, stopping at ±75°.)*
1b. *(Loop 32 done: rnd/harness.py analysis, D12, harness-kit segment table in harness.md.)* **Cable routing in CAD (build-enabling).** Model the harness:
    - paths through the hollow shafts (pitch Ø30, elbow Ø30, yaw Ø28 bores), service loops at each joint for its full range
    - the arm detach plug mount beside the V-band, the upper↔base coupling plug
    - torso e-tray ↔ shoulder ports, the head CAN/24 V drop along the mast

    Check bend radii and that every loop survives the joint range in the twin. Output a harness drawing (lengths, connectors) for a pre-made **harness kit**, which removes most crimping from the build (R10).
2. **R10 assembly time** (owner decision pending: one arm ≤ 4 h / robot ≤ 15 h). Levers beyond the harness kit:
    - snap-fit covers in place of screws on the cosmetic shells
    - one tool size (M3 / M4 hex) everywhere
    - print-in-place collars already done

    Re-estimate in BUILD_v4.md.
3. **Stiffness / hand-accuracy chain.** A static FEA chain with the CF tubes (bone shells + tubes + KX contact compliance) gives the hand deflection under a 1 kg load at full reach. Joint encoders don't see link bending. Confidence item at ~65 %.
4. **BOM headroom** ($96 under the $4,000 target). A cost pass:
    - bearing sourcing (28 × 6808 at ~$2 vs $4.7)
    - the wrist camera class
    - dock electronics
    - one fewer USB-CAN by merging buses?
5. **Electronics thermal:** the Pi 5 at ~73 % load inside the closed torso shell (Pi throttles at 85 °C). Vent path / fan sizing.
6. **Twin v2:** belt and bearing compliance in the dynamic sim; a second task (two-handed box lift); a battery-day sim with docking.
7. **Coupon round C1** (waits for the owner's P2S, ~30 days): package ready, now with the 72 h / 50 °C creep hold.
8. **Milestone M1** (owner buy decision, ≈ $1,100).
9. **Looks:** wheel fender lips; face animation set for the D11 screen; colour options.
10. **Research watch (every ~6 loops):**
    - GIM volume quote, DM-J4310 drawing
    - the Waveshare 4.3B real mass and active area
    - 6S BMS port type (common vs split) for the dock

Done since the confidence campaign:
- torque reaction 21/21 (loops 19–25)
- shoulder/roll/yaw wall re-derivation (21–22)
- elbow housing v2 (23)
- docking charger (26–27)
- D11 face screen (28)
- KX + threshold Monte Carlo (29–30)
- module clashes 0 with the head, base and upper included (31)

**Earlier queue history** is in [RND_LOG.md](RND_LOG.md).
