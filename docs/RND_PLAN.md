# Pip v4: hardware R&D plan

**Scope:** the **robot design** (mechanical, electrical, industrial design, simulation, cost). The AI stack comes later.

**Concept:** [V4_CONCEPT.md](V4_CONCEPT.md).

**How it runs:** a scheduled task works through the **Backlog** below, one item per run. Each run:

1. reads this file and [RND_LOG.md](RND_LOG.md)
2. does the next unchecked item
3. commits to `claude/robot-dog-p2s-printer-j4esie`
4. ticks the box
5. logs results, numbers, files and open questions

Items marked **⛔ GATE** pause for the owner's decision.

---

## 1. Requirements (measurable; every design decision traces back here)

| ID | Requirement | Target | Hard limit |
|---|---|---|---|
| R1 | **Reach** | Floor (z = 0) to 1.40 m; table, counter (0.90 m) and 1.2 m shelf | Floor to 1.25 m |
| R2 | **Payload per arm** | 1.0 kg at 45 cm reach; 2.5 kg two-handed near the body | 0.6 kg at reach |
| R3 | **Carry up stairs** | One adult carries it in **≤ 2 trips**, each piece **≤ 10 kg** with real handles. Splits/joins in **≤ 30 s with no tools.** "Carry mode" folds it compact. | Each piece ≤ 13 kg |
| R4 | **Carpet** | Drives and glides sideways on low and medium pile carpet, rugs, and 15 mm thresholds. Accurate position estimate on carpet. | Low-pile carpet + 10 mm thresholds |
| R5 | **Footprint** | Fits 70 cm doorways with arms folded; base ≤ 50 cm wide | ≤ 60 cm |
| R6 | **Speed** | 0.8 m/s base, smooth holonomic motion | 0.5 m/s |
| R7 | **Runtime** | 2 h light duty | 1 h |
| R8 | **Cost** (excluding Pi, filament, tools) | ≤ $3,300 | **$5,000** |
| R9 | **Reusable actuators** | Every motor in a **joint cartridge**; swapping or reusing one needs no special tools | – |
| R10 | **Assembly** | First build ≤ 4 h hands-on; reconfiguration ≤ 1 h | – |
| R11 | **Looks and motion** | Reads instantly as a *robot*: clean shells, expressed joints, light rings, expressive head, smooth motion | – |
| R12 | **Safety** | Back-drivable or torque-limited joints, e-stop on body and phone, 48 V protected | – |
| R13 | **Printability** | Every printed part fits the P2S (256 mm cube); no part needs supports where avoidable | – |

## 2. Architecture decisions so far (validate or overturn in the backlog)

| Decision | Hypothesis | Why | Validated by |
|---|---|---|---|
| **D1 Carry architecture** | **Two pieces + battery:** the *upper* (spine, lift, torso, arms, head) lifts off the *base* via one large kinematic coupling + cam lever + blind-mate 48 V/CAN connector; the battery slides out of the base. Budgets: upper ≤ 10 kg, base ≤ 6 kg, battery ≤ 3 kg. One "carry mode" command lowers the torso, folds the arms and tilts the head. Handles: spine top cap + base grips. | Meets R3 without stair-climbing; reuses the cartridge idea at robot scale | T02, T08 |
| **D2 Carpet drivetrain** | **3 × 6" (152 mm) omni wheels**, kiwi layout, direct-driven by GIM6010-8 cartridges. ≥ 25 mm ground clearance. Position from **optical-flow sensor + lidar**, not wheel turns (wheels slip on carpet). | FRC robots (up to ~125 lb) run 6" omnis on carpet; AndyMark's 6" DuraOmni is rated 120 lb with TPU rollers | T03 |
| **D3 Joint cartridges** | Kinematic coupling + bayonet + captive screw; two sizes: **M** (GIM6010-8 / DM-J4310) and **S** (STS3215); daisy-chain 48 V CAN-FD or 12 V TTL | Reusable investment (R9) | T06, T07 |
| **D4 Arm** | 6 DOF; shoulder pitch with a 2:1 printed belt + spring gravity assist; parallel gripper; wrist camera | R2 at lowest cost | T05 |
| **D5 Lift** | Belt + closed-loop stepper on aluminium extrusion, with a brake or self-locking hold when power is off | R1 cheaply | T09 |
| **D6 CAD toolchain** | **build123d** (Python, fillets, STEP/STL) for v4 parts; Blender renders; MuJoCo twin generated from the same parameters | R11 looks + one source of truth | T01b |

## 3. Workstreams

| WS | Name | Output |
|---|---|---|
| WS1 | Specs and suppliers | Verified datasheet numbers (mass, torque, speed, dimensions, protocol, price) in `rnd/specs/` |
| WS2 | System architecture | Mass budget, power budget, carry split, interfaces |
| WS3 | Simulation twin | MuJoCo model of v4 from build123d parameters: reach, torque, base on carpet (friction model), tipping |
| WS4 | Joint cartridge standard | CAD + print-test plan + tolerance study |
| WS5 | Arm | Kinematics, sizing, CAD, gripper |
| WS6 | Torso + lift + head | CAD, brake, cable management |
| WS7 | Base + carry | CAD, detach coupling, battery bay, handles |
| WS8 | Power and electronics | 48 V bus, e-stop, BMS, DC-DC, CAN topology, wiring diagram |
| WS9 | Industrial design | Shells, colour, light design, renders |
| WS10 | Cost, BOM, assembly | Living BOM, cost roll-up, assembly sequence and time |

## 4. Backlog (in order; one item per run)

Each item has a deliverable and a "done when". If an item is too big for one run, split it and log the split.

- [ ] **T01: Verified specs (WS1).** GIM6010-8, DM-J4310, DM-J4340, DM-J8009P, STS3215 (12 V), 6" omni wheels, 48 V e-bike batteries (7–15 Ah), closed-loop NEMA 23, RGB-D candidates (Orbbec / RealSense), USB-CAN-FD adapters. Record mass, dimensions, rated/peak torque, speed at 24/48 V, voltage, protocol, mounting pattern, price, links. → `rnd/specs/actuators.yaml`, `rnd/specs/components.yaml`. *Done when* every number has a source link or is marked ESTIMATE.
- [ ] **T01b: Toolchain.** `rnd/` Python package: build123d parameter file (`params.py`), part-module convention, STL/STEP export script, Blender render script that loads exported meshes, MuJoCo builder stub. *Done when* `python -m rnd.export` and a test render run.
- [ ] **T02: Mass and power budget + carry split (WS2).** Spreadsheet-style `rnd/budget.py` → `docs/rnd/budget.md`: mass per module, CoM, power per mode, runtime. Check D1 against R3/R7; propose battery size. *Done when* the budgets close or a requirement conflict is logged.
- [ ] **T03: Carpet drivetrain trade study (WS7).** Compare kiwi 3 × 6" omni, X-drive 4 × 4" omni, mecanum, and differential + casters on carpet (rolling resistance, threshold climb, lateral motion, vibration, cost, odometry). Pick wheels, clearance and odometry sensors; check motor torque and speed vs R4/R6. → `docs/rnd/drivetrain.md`. *Done when* D2 is confirmed or replaced.
- [ ] **T04: MuJoCo twin v0 (WS3).** Simplified v4 model (base, lift, two 6-DOF arms, head) from `params.py`; reach study for R1 (floor, 0.75 m table, 0.90 m counter, 1.2 m shelf, dishwasher rack) and tip-over margins with 1 kg at reach while lifted. → `docs/rnd/twin_v0.md` + images. *Done when* reach maps and tipping results are logged.
- [ ] **T05: Arm sizing (WS5).** Link lengths, joint layout, required torques at each joint (static + dynamic) vs cartridge ratings; spring-assist and belt-ratio design; gripper stroke and force. → `docs/rnd/arm_sizing.md`. *Done when* each joint has a cartridge with ≥ 1.5× margin or a logged exception.
- [ ] **⛔ GATE A: owner review** of budgets, drivetrain, twin and arm sizing (write a short summary + 3 renders/plots, list decisions).
- [ ] **T06: Cartridge standard v0 CAD (WS4).** M and S cartridges + bone interface (kinematic balls/vees, bayonet, detent, thumbscrew, connector pocket, cable path). Print-test plan with tolerance coupons. → `rnd/parts/cartridge_*.py`, STLs, `docs/rnd/cartridge.md`.
- [ ] **T07: Cartridge validation in sim/FEA-lite.** Load cases from T05: bayonet and ball-seat stresses (hand calc + simple FEA if available), print orientation, expected repeatability. Refine T06.
- [ ] **T08: Base + carry CAD (WS7).** Base chassis, wheel cartridges, battery bay (slide-out), upper/base coupling + cam lever + blind-mate connector, handles, bumper/edge lights.
- [ ] **T09: Lift + spine CAD (WS6).** Extrusion, carriage, belt, stepper, brake or hold, cable chain, top handle cap.
- [ ] **T10: Torso + shoulders CAD (WS6).** Shoulder cartridge mounts, electronics bay (Pi 5, CAN, DC-DC), cooling.
- [ ] **T11: Arm CAD (WS5).** Bones for 6 joints, spring-assist module, gripper with wrist camera.
- [ ] **T12: Head v4 CAD (WS6).** Visor, display eyes, RGB-D, mic array, speaker, pan/tilt S cartridges.
- [ ] **T13: Power and electronics (WS8).** Single-line diagram, e-stop chain, fusing, BMS interface, DC-DC sizing, CAN-FD topology and termination, connector standard. → `docs/rnd/electrical.md` + diagram.
- [ ] **T14: Twin v1 from real CAD (WS3).** Regenerate MuJoCo from the build123d parts (masses from geometry), re-run reach, torque, tipping and carpet-drive tests.
- [ ] **T15: Industrial design pass (WS9).** Shell surfaces, colour and material, light language, carry-mode pose; Blender hero renders (3 views + carry mode + exploded).
- [ ] **T16: BOM, cost and assembly (WS10).** Living BOM with links, cost vs R8, assembly sequence and time vs R10.
- [ ] **⛔ GATE B: owner design review** (renders, cost, risks).
- [ ] **T17: Phase-1 desk-arm build package.** STLs, BOM and order list, assembly guide for one arm on a desk mount using production cartridges.
- [ ] **T18: Close-out.** Update V4_CONCEPT.md, README; summarize; disable the scheduled task.

## 5. Rules for each run

- Work **only on robot design**, not the AI stack.
- Prefer verified numbers. Mark estimates `ESTIMATE` and say how to verify them.
- Keep every part ≤ 256 mm (P2S), design for straight-down assembly, and use the cartridge interface for every actuator.
- Commit with clear messages, push to the branch, append to `docs/RND_LOG.md` (date, item, results, files, open questions).
- At a ⛔ GATE: write the summary, push, then stop and don't continue past the gate until the owner answers in the conversation.
- Don't buy anything, don't open PRs, and don't change the AI/software stack except for sim tooling.
