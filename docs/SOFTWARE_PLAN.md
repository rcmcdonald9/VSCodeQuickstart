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
| **S0.1** | `sw/bench_env.py`: MuJoCo bench-arm environment (one arm on a stand, table, objects; wrist + exterior cameras; EEF-delta action interface via our IK), gymnasium API | no |
| **S0.2** | Robot description export (URDF + MJCF + meshes from the CAD) for ROS 2 / MoveIt / LeRobot | no |
| **S0.3** | `sw/gen_demos.py`: scripted expert (the twin's IK pick-and-place planner), domain-randomised (lighting, textures, object pose, camera jitter) → **synthetic demos in LeRobot format** | no |
| **S0.4** | Training-pipeline dry run: load a pretrained small policy, co-train on synthetic + an open-dataset slice (CPU smoke test; real runs on a GPU later) | no (GPU later) |
| **S1** | Bench bring-up: drivers (GIM6010 CAN, DM-J4310 MIT mode, STS3215 TTL, RP2040 encoder nodes), 500 Hz control loop, calibration, gravity compensation with the balancer | M1 kit |
| **S1-safety** | Safety supervisor (independent of the policy): torque/speed/workspace limits, watchdog, e-stop → torque-off | M1 kit |
| **S2** | Teleop and recording: EEF teleop (leader arm through IK, or VR hand tracking); LeRobot recorder | M1 kit + leader arm |
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
