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
| R8 | Cost (excluding Pi, filament) | ≤ $3,300 | $5,000 |
| R9 | Reusable actuators | All motors in joint cartridges | – |
| R10 | Assembly | First build ≤ 4 h hands-on; reconfiguration ≤ 1 h | – |
| R11 | Looks and motion | Reads as a robot: clean shells, expressed joints, light rings, expressive head, smooth motion | – |
| R12 | Safety | Torque-limited joints, e-stop, protected 24 V, stable (tip margin ≥ 50 mm under worst static load) | – |
| R13 | Printability | Parts ≤ 256 mm, support-free where possible | – |

## 3. Decisions (living; reversals logged)

| # | Decision | Status | Evidence |
|---|---|---|---|
| D1 | **Carry split:** upper (spine, lift, torso, arms, head) / base / battery, with a no-tools coupling and handles | Holding: upper 9.5 kg, base 5.8 kg (tight), battery 1.5 kg | [budget](rnd/budget.md) |
| D2 | **Base: 4 × 6" omni, one wheel pointing forward, wheels at R 0.21 m, Ø500 shell.** *Reversed from 3-wheel kiwi:* kiwi tip margin was ~35–46 mm vs 66–82 mm | Changed 2026-11-25 | [twin](rnd/twin.md) |
| D2b | Cross thresholds at a 45° heading; cap base acceleration at 1.0 m/s² with arms parked and 0.5 m/s² while manipulating; position from optical flow + IMU + lidar | New | [drivetrain](rnd/drivetrain.md) |
| D3 | Joint cartridges: kinematic coupling + bayonet + captive screw; M (GIM6010-8 / DM-J4310) and S (STS3215); 24 V CAN 1 Mbps / 12 V TTL | Holding | [T01](rnd/T01_specs.md) |
| D4 | **Arm: 0.30 + 0.28 + 0.13 m, wrist ±120°** (loop 1: ±140° folded the hand into the forearm; ±120° gives more collision-free reach). **CAD requirement:** forearm bone ends ≥ 60 mm before the wrist axis. Shoulder pitch GIM × 2.5 belt + 85 % spring, roll GIM × 3, upper-arm yaw DM-J4310, elbow GIM × 2, wrist pitch **DM-J4310**, wrist roll and gripper STS3215. *Changed* from 0.25 + 0.23, ±110°, wrist STS (reach coverage 31 % → 100 %; STS wrist was under-rated) | Changed 2026-11-25 | [arm sizing](rnd/arm_sizing.md), [twin](rnd/twin.md) |
| D5 | **Lift: shoulder height 0.35–1.05 m** (0.70 m travel); belt + stepper, with a brake or counterbalance (study pending) | Range changed (was 0.40–0.95) | [twin](rnd/twin.md) |
| D6 | Toolchain: build123d CAD, MuJoCo twin generated from `rnd/params.py`, Blender renders | Holding | `rnd/` |
| D7 | **Power bus 24 V (6S Li-ion)**, replacing 48 V | Holding | [T01](rnd/T01_specs.md) |

## 4. Work queue (ordered; edit freely)

Done so far: T01 specs ✅, toolchain v0 (`rnd/`) ✅, budgets v0 ✅, drivetrain v0 ✅, twin v0 reach and tip-over ✅, arm sizing v0 ✅.

1. ~~Twin: collisions~~ ✅ loop 1: collision-free IK, wrist ±120°, bimanual 4/4.
1b. **Counter top-down reach (62 %)**: evaluate a **two-stage telescoping spine** (forklift-mast style) that reaches shoulder 1.25 m while staying ≤ 1.1 m long for carrying; compare with a taller single spine and a higher shoulder mount. Pick by reach, carry and mass.
2. **Cartridge standard v0 CAD** (build123d): M and S cartridges, kinematic coupling + bayonet + detent + connector + cable path; tolerance-coupon prints. First find the GIM6010-8 mounting drawing.
3. **Arm CAD v0:** bones with printed belt reductions (2–3:1), shoulder spring-assist module, wrist, gripper with camera.
4. **Base CAD v0:** 4-wheel diamond chassis, wheel cartridges, slide-out battery, upper/base coupling + cam lever + blind-mate connector, handles, bumper light.
5. **Lift and spine:** compare a constant-force-spring counterbalance (smaller motor, safe if power is lost) against a brake motor; CAD for the carriage, cable chain and top handle.
6. **Torso + head CAD:** electronics bay (Pi 5, CAN, DC-DC), visor, eyes, depth camera, mic, speaker, pan/tilt.
7. **Electrical architecture:** 24 V single-line diagram, e-stop chain, fusing, BMS, DC-DC sizing, CAN topology, connector standard.
8. **Twin v1 from CAD:** masses and inertias from the geometry; **dynamic** MuJoCo with actuator models (pick-and-place motion, base acceleration, carpet friction); re-verify the scorecard.
9. **Industrial design pass:** shells, colour and material, light language, carry mode, exploded view; Blender hero renders.
10. **BOM, cost and assembly time;** carry split validated from CAD masses.
11. **Milestone M1: Phase-1 desk-arm build package** (cartridges + one arm + desk mount + leader arm). Ask the owner: buy?
12. **Research watch (recurring, every ~6 loops):** new or cheaper actuators, better precedents, price changes. Challenge the big decisions (two arms vs one, lift vs taller arms, omni vs other drive).

**Milestones:** M1 desk-arm package → M2 full-robot CAD + twin v1 → M3 complete build package (STLs, BOM, assembly guide, wiring).
