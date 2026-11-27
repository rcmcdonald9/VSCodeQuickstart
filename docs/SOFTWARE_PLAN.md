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
| **S1-safety** ✅ software (loop 40) | `sw/safety.py` + `sw/test_s1.py` (fault injection in the twin). Safety supervisor (independent of the policy): torque/speed/workspace limits, watchdog, e-stop → torque-off | M1 kit |
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
