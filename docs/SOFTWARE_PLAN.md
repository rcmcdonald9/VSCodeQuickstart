# Pip v4: software track (owner, 2026-11-26)

**Scope:** software for the robot, starting with the **M1 bench arm** (one arm on a stand). It runs alongside the hardware
R&D loop (see RND_PLAN §1 for the alternation and the efficiency rules). Everything is built from `rnd/params.py`, so the
software and the hardware design stay in sync.

**Strategy (the speed lever is data):** don't train from scratch. Start from a **pretrained open policy**, **co-train with
open robot datasets**, add **synthetic demonstrations from our own MuJoCo twin**, and spend real teleoperation demos only on
the gap. Design every interface to look like the large open datasets, so their data and checkpoints transfer:

| Choice | Convention we adopt | Why |
|---|---|---|
| Action space | **End-effector pose deltas (6D) + gripper**, 10–15 Hz policy rate, chunked; joint targets come from our IK | DROID / Open X-Embodiment / most VLA checkpoints use EEF-delta actions. Our 6-DOF + gripper arm differs from every dataset robot in joint space, but not in EEF space |
| Cameras | **Wrist camera + one exterior camera** (+ the head RGB-D on the full robot), 640×480-class, 15–30 fps | DROID's layout (wrist + exterior) |
| Data format | **LeRobot dataset format** (parquet + mp4) | The common interchange format; LeRobot has loaders for the hub datasets and training scripts for ACT, Diffusion, SmolVLA, π0-family |
| Language | One instruction string per episode | Needed for VLA fine-tuning and multi-task |

## Open resources to leverage (shortlist; S3.1 benchmarks them on our task)

- **Pretrained policies:**
  - π0 / π0.5 family (Physical Intelligence `openpi`)
  - SmolVLA (Hugging Face; small, trains on one GPU; pretrained on community LeRobot data)
  - NVIDIA GR00T N-series
  - OpenVLA / OpenVLA-OFT (7B; trained on 970k Open-X trajectories; fine-tuned with 10–150 task demos)
- **Datasets for co-training:**
  - Open X-Embodiment (≈1M trajectories, 22 robots)
  - DROID (Franka, wrist + exterior cameras, EEF actions: the closest match to our convention)
  - BridgeData V2
  - LeRobot hub community datasets (SO-100/SO-101 arms: cheap printed arms like ours)
  - large 2025–26 releases (e.g. AgiBot World-class bimanual data, for the later two-arm phase)
- **Human video:** egocentric glasses data for co-training later (EgoMimic-style). Pip's head camera sits near human eye height, which helps.
- **Caveats (checked in S3):**
  - Fine-tuning a generalist on few demos can cause forgetting, so co-fine-tune with a slice of open data.
  - A checkpoint's action-space conventions (frames, scaling, gripper sign) must be matched exactly before running it on hardware (deployment-safety literature, 2026).

## Phases and deliverables

| Phase | Deliverable | Needs hardware? |
|---|---|---|
| **S0.1** ✅ | `sw/bench_env.py`: MuJoCo bench-arm environment (one arm on a stand, table, objects; wrist + exterior cameras; EEF-delta action interface via our IK), gymnasium API | no |
| **S0.2** ✅ (loop 40) | Robot description export: `sw/export_description.py` → URDF + MJCF + joints.json (limits, ratings) from the same model as the sims; CAD meshes still to add | no |
| **S0.3** ✅ (loop 40; v3.0 via the official writer, loop 42-sw: `sw/to_lerobot_v3.py`, reloaded with `LeRobotDataset`) | `sw/gen_demos.py`: scripted expert (the twin's IK pick-and-place planner), domain-randomised (lighting, textures, object pose, camera jitter) → **synthetic demos in LeRobot format** | no |
| **S0.4a** ✅ (loop 40) | `sw/train_bc.py`: dataset → decode → train a small BC policy from scratch on CPU → closed-loop eval in the twin ([train_bc.md](sw/train_bc.md)) | no |
| **S0.4b** ⛔ | Pretrained small policy (SmolVLA / π0-family) co-trained on synthetic + an open-dataset slice. **Blocked: this environment's network policy refuses huggingface.co** (owner decision to allow it) | no (GPU later) |
| **S1** (bring-up tool ready, 2026-11-27) | `sw/bringup.py` M1 day-1 bring-up on python-can (`--bus virtual` now, `socketcan:can0,can1` on day 1): enumerate → drive limits → e-stop/heartbeat/CAN-timeout chain → hard-stop calibration → 10 % jog → gravity hold (residual, friction, drive-clamp sag test, collision thresholds) → report; each step refuses to run unless the previous passed, supervisor in the loop. DM-J4310 path complete (`sw/drivers/mit.py` table); GIM6010 = TODO table in `sw/drivers/gim.py` (fill from the manual on day 1). Tested end to end against `sw/virtual_arm.py` (MuJoCo plant + simulated drives and stop circuit, fault injection): [s2_tests.md](sw/s2_tests.md). Still open: RP2040 encoder nodes, STS3215 TTL, 500 Hz loop on the Pi (SW1) | M1 kit |
| **S1.0** GIM protocol discovery (Fable M4, 2026-11-27) | `sw/gim_discover.py`: bring-up step 1 on the ONE Step-1 GIM6010-8 (listen, no-torque MIT exit-motor-mode probe, optional zero-gain motor-mode trace) → docs/sw/gim_discovery_<date>.md; what is public about the SDC104 / SDC303 / GIM8108-8 drivers: [gim_protocol_notes.md](sw/gim_protocol_notes.md) and `GIM_KNOWN` in `sw/drivers/gim.py`. Rehearsed against a simulated MIT-family drive in `sw/test_s2.py`. Its result decides whether the supervisor's torque bound can hold on the GIM joints | M1 kit Step 1 |
| **S1.1** watchdog sized on jitter (Fable M2, 2026-11-27) | `sw/loop_jitter.py`: wall-clock run of `BringUp.cycle` (RealClock, supervisor, codec, python-can, 5 joints): longest heartbeat gap 5.5 ms (7.9 ms with the cores loaded) on this machine → hardware watchdog 8 → 25 ms, drive CAN timeout 20 → 30 ms (`sw/config/bringup.yaml` `watchdog_s`, `can_timeout_s`; checked in electrical.md against SW2); the bring-up report logs the real loop's longest gap. Re-run on the Pi before day 1 | M1 kit Step 1 |
| **S1.2** bus realism (Asimov sim2real lessons, 2026-11-27) | `sw/bus_sim.py`: the virtual bench's CAN now delays (0–9 ms, FIFO, so frames bunch), drops, duplicates, CRC-rejects (error counters, bus-off) and corrupts frames; `sw/bus_realism.py` runs bring-up, supervisor and collision observer per profile → [bus_realism.md](sw/bus_realism.md). Found and fixed: the 10 ms stale-feedback bound false-trips at 0–9 ms → **20 ms** (`fb_max_age_s`); a stale buffered frame accepted as a fresh answer; CLEAR_ERROR in flight read as a drive fault; register waits that blocked the loop without a heartbeat; one corrupted position bit stopped calibrate → driver plausibility gate. Interface choice: [can_interface_note.md](sw/can_interface_note.md) (`sw/can_latency.py`) | no (measure on Step 1) |
| **S1.3** actuator sysid / real2sim (2026-11-27) | bring-up step 7 `sysid` (`sw/sysid.py`): friction sweeps (Coulomb + viscous), torque constant from sweeps with a known test mass, backlash at direction reversals, command → response latency (steps + chirp) → `sw/config/actuators_<serial>.yaml`, loaded by `rnd/twin.py actuator_model()`, `sw/bench_env.py`, `sw/virtual_arm.py` (datasheet model otherwise). Rehearsed on the Step-1 rig with hidden values: all within 5 % (latency 1–2 % on USB, ≈ 10 % at 0–9 ms) → [sysid_rehearsal.md](sw/sysid_rehearsal.md). Measured values are not domain-randomised; payload and contact friction still are | M1 kit Step 1 |
| **S1.4** determinism + soak (2026-11-27) | `sw/determinism.py` in every bring-up report: loop-period histogram, CPU temperature + `vcgencmd get_throttled`, PREEMPT_RT / scheduling, CAN error counters and bus-off. `sw/soak.py`: sysid → 2 h at ≈ 50 % rated → sysid; drift limits (Coulomb 20 %, viscous 25 %, K 5 %, backlash +0.1°, latency +0.5 / +2 ms p95), a Step-1 exit criterion (M1_READINESS 7). Summary: [sim2real_bench.md](sw/sim2real_bench.md) | M1 kit Step 1 |
| **S1-safety** ✅ software (loop 40) | `sw/safety.py` + `sw/test_s1.py` (fault injection in the twin). Safety supervisor (independent of the policy): torque/speed/workspace limits, watchdog, e-stop → torque-off | M1 kit |
| **S1-safety.1** people-near mode (D25 / R17, 2026-11-27) | `Supervisor.set_people_near(on)` in `sw/safety.py`: while a person is close the shoulder roll is held at **≥ +5°** (targets ≥ +8° with the supervisor margin; `people_near` range in `sw/description/joints.json`, from `rnd/twin.JR_PEOPLE_NEAR`; rnd/pinch.py finds the inboard roll wedge ≥ 25 mm from ≈ +1° and 30 mm at +5°, see the people-near column and limit scan in [pinch.md](rnd/pinch.md)); targets below it are clamped like any joint limit. Switching the mode on with the arm in the zone does not stop it: the bound starts at the arm and rises at 0.2 rad/s, so the arm is walked out (≈ 1.6 s from −10°); the faults still use the hard limits. Off → the full −15…+100° range at once. **Trigger** (`PeopleNearTrigger`, a stub until perception exists): on as soon as a person is within **1.5 m** of the mast axis, from the head RGB-D person detector fused with the lidar leg tracker (the base lidar sees legs all round, the head camera covers the front and confirms a person rather than a chair); off only after nobody has been within 1.8 m for 2 s (hysteresis); no perception input, or one older than 0.5 s, counts as "person near" (fail-safe). The planner should keep across-the-body work for when nobody is close (R16 cost with the mode on, [dexterity.md](rnd/dexterity.md): across the body 71 → 49 %, top-down 35 → 18 %, 45° 78 → 57 %, side 78 → 67 %). Tested in `sw/test_s1.py` (random policy with the mode on: 0 cycles below the limit; mode switch in the zone: slewed exit, no stop) | perception (S4) for the real trigger |
| **S2** (simulated part ready, 2026-11-27) | Teleop and recording. `sw/teleop_sim.py`: EEF-delta teleop of the bench twin (keyboard in the MuJoCo viewer, or a scripted leader headless) through the supervisor at 500 Hz, recorded as LeRobot v3.0 with the same writer as `sw/to_lerobot_v3.py`; e-stopped episodes discarded. Still to do on hardware: leader arm through IK (or VR hand tracking), real cameras, SW3 latency | M1 kit + leader arm |
| **S3.1** | Checkpoint shortlist benchmark on the bench task (zero-shot / 10 / 50 demos) | GPU (owned or rented) |
| **S3.2** | First skill: pick-and-place a cup | GPU |
| **S4** | Full robot: ROS 2 Nav2 + lidar SLAM, lift, bimanual, docking (AprilTag + V-dock), D11 screen UI | full robot |

## Software targets (measurable, scorecard rows once they exist)

| ID | Target |
|---|---|
| SW1 | Control loop 500 Hz on the Pi 5, p99 jitter < 1 ms, with the cameras streaming |
| SW2 | E-stop or watchdog → all torque off in < 50 ms; policy output always passes the safety supervisor |
| SW3 | Teleop end-to-end latency < 100 ms |
| SW4 | First skill (pick-and-place cup) ≥ 80 % success with **≤ 50 real demos** (from-scratch baseline ≈ 200) |
| SW5 | Policy inference ≥ 10 Hz on the Jetson (Orin Nano Super class) |
| SW6 | Sim pipeline: ≥ 500 synthetic demos/hour on this machine; sim success of the scripted expert ≥ 95 % |
