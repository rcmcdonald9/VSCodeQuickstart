# Pip v4 R&D log

Newest entries at the bottom. Each entry: date (UTC), backlog item, what was done, key numbers, files, open questions for the owner.

---

## 2026-11-25: plan created

- Wrote [RND_PLAN.md](RND_PLAN.md): requirements R1–R13, including the new **carry up stairs** (R3) and **carpet** (R4) requirements; architecture hypotheses D1–D6; 18-item backlog with two owner gates.
- **New-requirement decisions (to validate):**
  - **Carry:** the robot splits into upper + base + battery (≤ 10 / 6 / 3 kg targets), with a no-tools coupling and a "carry mode".
  - **Carpet:** 3 × 6" omni wheels (FRC-proven on carpet; AndyMark 6" DuraOmni rated 120 lb with TPU rollers). Position from optical flow + lidar.
- Toolchain check: build123d installs and works in this environment (fillets and STL export tested).
- **Next:** T01, verified specs.
