# Simulator options for later (owner note, 2026-11-27)

Source: Menlo Research, "How we achieved zero-shot sim2real" (Asimov humanoid, Aug 30 2026; pasted by the owner — the
host is blocked from this environment).

What Menlo uses:
- **mjlab** (MuJoCo-based) for fast iteration: early-stage methodology validation, parameter tuning and "wilder" policy
  experiments where iteration speed matters more than fidelity.
- **Isaac Lab** for later-stage physics learning, "especially relying on the Newton physics integration".
- **sim2sim** across the two (train in one, check in the other) before going to hardware.
- Their actuator models are published as configs in an Isaac Lab pull request (worth reading for how they model
  actuators instead of randomizing around them).

Where Pip stands: one simulator, MuJoCo on CPU (rnd/twin.py, sw/bench_env.py, sw/virtual_arm.py), which suits the
current work (statics, IK, scripted demos, BC, the bring-up rehearsal). Revisit when we train policies at scale
(S3/S4 in SOFTWARE_PLAN.md):
1. mjlab first: same MuJoCo models (our MJCF export, sw/export_description.py), so it is the cheaper step.
2. A sim2sim check (MuJoCo ↔ a second engine) before trusting any learned policy on the arm.
3. Isaac Lab needs an NVIDIA GPU: that means GPU rental or hardware, which is an owner decision (no GPU rental
   without the owner's OK).
Not verified here: mjlab's and Isaac Lab/Newton's current install requirements; check when we get there.
