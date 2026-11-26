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
| R8 | Cost (excluding Pi) | ≤ $4,000 (was $3,300; raised for the D9 sensor pack, owner 2026-11-25) | $5,000 |
| R9 | Reusable actuators | All motors in joint cartridges | – |
| R10 | Assembly | First build ≤ 4 h hands-on; reconfiguration ≤ 1 h | – |
| R11 | Looks and motion | Reads as a robot: clean shells, expressed joints, light rings, expressive head, smooth motion | – |
| R12 | Safety | Torque-limited joints, e-stop, protected 24 V, stable (tip margin ≥ 50 mm under worst static load) | – |
| R13 | Printability | Parts ≤ 256 mm, support-free where possible | – |

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
| D6 | Toolchain: build123d CAD, MuJoCo twin generated from `rnd/params.py`, **arm collision geometry from CAD hulls** (`rnd/parts/collision.py`, loop 6), Blender renders | Holding | `rnd/` |
| D4c | **Belt stages: HTD 5M, 15 mm, 20T small pulley** (pitch 20:50, roll 20:60, elbow 20:40), printed pulleys, the small pulley on its own bearing hub. **Elbow motor stays at the elbow:** a remote motor on a long belt (Blue-style) was evaluated and rejected (–0.5 N·m at the shoulder, +0.24 kg) | New (loop 3) | [belts](rnd/belts.md) |
| D7 | **Power bus 24 V (6S Li-ion)**, replacing 48 V | Holding | [T01](rnd/T01_specs.md) |

## 4. Work queue (ordered; edit freely)

**Milestone M2 reached (2026-11-25 build-out session):**
- every module in CAD; full assembly with 0 cross-module clashes; 59/59 printed pieces fit the P2S
- dynamic twin v1 completes a drive + floor pick + table place within real actuator limits (8/8 robust)
- electrical plan, itemised BOM ($3,220), print plan (10.4 kg, ~255 h P2S) and build guide (~25 h hands-on)
- see [BUILD_v4.md](BUILD_v4.md)

Next (state after the confidence campaign; the hourly loop continues from here). **Items 1–3 come first: they are real design gaps found by the new checks.**
1. **Torque-reaction path for every cartridge** ([integration.md](rnd/integration.md): **11/21 after loop 22**; wheels: standard `cm.torque_hanger`; lift, shoulder pitch, roll, yaw: forks integral to the mount / bracket / roll link / clevis). Next: elbow ×2, wrist pitch ×2, wrist roll ×2, head pan + tilt. Remaining from the original list: roll ×2, yaw ×2, elbow ×2. The wrist-pitch and head-tilt pins currently *run into* a wall (no slot): cut a radial slot there. Each can's Ø6 rear torque pin needs a radial slot in structure. Proposed standard part:
    - an axial key rib along every can, engaging a radial notch in a printed **bay ring** on the nearest wall/plate
    - or, where the can sits in free space, a **torque strap** from the nearest wall to the can rear
    - stages: pitch ×2, roll ×2, yaw ×2, elbow ×2, wrist pitch ×2, wrist roll ×2, wheels ×4, lift, head pan
    - FEA each slot at peak motor torque (GIM 11 N·m at r 30 → 367 N)
    - make `rnd.integration` pass
2. ~~Shoulder pitch walls re-derived from the hub geometry~~ (done in loop 21).
3. **Elbow: collar vs housing top/corners (1.8 cm³) and forearm hub disc vs the +Y elbow boss (2.7 cm³, rotating).**
    - Collar Ø98 at the cartridge bay: housing too small. Raise HOUSING z1, or reduce the corner radius on the −Y side.
    - Forearm: move FORE_Y out ~2 mm, or recess the boss.
    - Then design the elbow bay ring for item 1.
4. **Self-docking charger (owner request, 2026-11-26).** Goal: the robot charges itself whenever idle, so the 3.6 h runtime rarely matters.
    - **Contacts:** two spring-loaded contacts at the back of the base skirt, plus a third pin (ID/handshake). The dock's terminals stay dead until the robot identifies itself, so no bare live contacts at floor level. Reverse-polarity protection and a fuse on the robot side.
    - **Dock (printed):** a funnel/V-guide that self-centres the base within ±30 mm and ±10°, a weighted or screw-down foot, and a low profile so it doesn't block the lidar.
    - **Charger:** 6S Li-ion CC/CV to 25.2 V, sized ~4–5 A for a ~2.5 h full charge from empty (ESTIMATE). Check that the pack/BMS (Heltec 6S) takes charge through the same port while the robot is powered (load sharing).
    - **Finding the dock:** a lidar-visible profile on the dock face (precedent: robot vacuums, TurtleBot 4 / Create 3 dock) and an AprilTag for the head camera. The approach runs at threshold speed (≤ 0.15 m/s).
    - **Checks:** MuJoCo approach Monte Carlo (start pose ±0.3 m / ±20°, wheel slip), contact-force FEA, clashes with the skirt and wheels, BOM and wiring-plan lines, a scorecard row. Battery stays ≥ 15 % when it heads home.
5. Remaining confidence-campaign sims: KX Monte Carlo (tolerance, μ, spring) + PETG creep under the KX preload and belt tension; threshold/carpet Monte Carlo.
6. **R10 assembly time** (22 h vs 4 h, the one failing metric). Owner decision pending on splitting the target (one arm ≤ 4 h / robot ≤ 15 h). Levers: fewer distinct printed parts, snap-in cartridge cans, harness kits.
7. **Coupon round C1: package READY** ([COUPONS.md](COUPONS.md), loop 18). It needs the owner's P2S and ~$25 of small parts:
    - print-in-place KX socket, eccentric bushing, GIM bolt-circle coupon
    - HTD pulley vs a real belt, ratchet + pawl
    - fin-ray finger (TPU)
    - measure, then replace the ESTIMATEs
8. **Milestone M1 (buy decision for the owner):** priced at ≈ $1,100 ([M1_BENCH_ARM.md](M1_BENCH_ARM.md)); everything except the bench PSU is reused in the full robot.
9. **Twin v2:** belt/bearing compliance in the dynamic sim; a second task (two-handed box lift); a battery-day simulation.
10. **Looks:** wheel fender lips; head face/eye light language; colour options for owner approval.
11. **Research watch (every ~6 loops):** GIM volume quote, DM-J4310 drawing, sensor prices (D9 lines are ESTIMATEs).

**Earlier queue history** is in [RND_LOG.md](RND_LOG.md).
