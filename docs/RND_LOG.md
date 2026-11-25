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

## 2026-11-25: T01 verified specs ✅

- Collected specs with sources for GIM6010-8, DM-J4310 / J4340P / J8009P, STS3215-C018, 6" DuraOmni, 6S and 48 V batteries, RGB-D cameras, wrist cameras, 2D lidar, optical flow, CAN adapters, NEMA 23 lift motors → `rnd/specs/*.yaml`, summary [rnd/T01_specs.md](rnd/T01_specs.md).
- **Key finding → new decision D7:** power bus **24 V (6S)**, not 48 V. GIM6010-8 manual range is 12–28 V. A 6S 10 Ah pack is ~1.5 kg vs 2.7–4 kg for 48 V, which helps R3.
- Drivetrain on paper: GIM6010-8 direct on a 6" omni gives ~66 N per wheel and 0.96 m/s at rated speed (R6 met).
- Shoulder torque is likely tight with one GIM6010-8 → T05 decides between belt + spring and a DM-J4340P/J8009P upgrade.
- The motors use classic CAN at 1 Mbps (no FD needed). Wrist-camera budget raised (good ones cost $60–155).
- **Open questions for owner:** none blocking. Nice-to-know: do you prefer quieter + slower (belt/lead-screw lift) or faster lift?
- **Next:** T01b toolchain.

## 2026-11-25: operating model → continuous loop; toolchain + budgets + drivetrain + twin + arm sizing (v0)

**Owner direction:** Claude runs development, iterates long, changes course when evidence says so. [RND_PLAN.md](RND_PLAN.md) is rewritten as a continuous loop (critique → work → measure → decide → record → digest) with a living work queue and a reversible decision table. Gates removed; the owner is only needed for buy decisions.

**Built:** `rnd/` package.
- `params.py`: single source of truth.
- `budget.py`, `arm_sizing.py`, `drive.py`.
- `twin.py`: MuJoCo model generated from params; IK reach study; tip-over margins per wheel layout; renders.
- `run_all.py`: regenerates every report and the [scorecard](rnd/SCORECARD.md) in ~20 s.

**Findings and decisions (before → after):**

| Finding | Before | After | Decision |
|---|---|---|---|
| Reach coverage (IK, 6 household target sets) | 31 % | **97 %** | Arm 0.25 + 0.23 → **0.30 + 0.28 m**, wrist ±110° → **±140°**, shoulder height 0.40–0.95 → **0.35–1.05 m** (sweep of 13 variants) |
| Tip-over margin, 1 kg at reach | kiwi: **35–46 mm** (only ~0.8 m/s² braking with a high reach) | 4-wheel, one wheel forward: **66–82 mm** | **D2 reversed: kiwi → 4 × 6" omni**, one wheel forward, R 0.21; base faces the target; acceleration caps 1.0 / 0.5 m/s² |
| Joint margins at spec load | shoulder roll 1.16×, wrist STS under-rated | **min 1.70×** | Belts: pitch 2.5:1, roll 3:1, elbow 2:1; wrist pitch STS → **DM-J4310** |
| Mass (carry) | – | upper 9.5 / base 5.8 / battery 1.5 kg (total 16.8) | D1 holds; base is tight |
| Cost | – | ~$2,800 (actuators $1,550 for 20 joints) | Within R8 |
| Runtime | – | ~4 h (estimated duty cycle) | Within R7 |
| Thresholds | – | 15 mm needs 3.7 N·m per wheel at a 45° heading | Planner crosses at 45°; **bench test**; fallback 8" dual-row omni |

**Bugs found and fixed along the way:** MuJoCo hinge ranges were double-converted (radians); arm joint sign conventions were wrong; the IK solver stopped too early (added structured starts and more iterations).

**Open for future loops:** the reach study has no self/base collision checks yet (queue #1); omni-roller threshold behaviour needs a physical test; rolling resistance and traction on carpet are estimates.

**Next:** queue #1, then cartridge CAD.

## 2026-11-25: loop 1, collision-aware reach + bimanual

**Picked:** queue #1. The 97 % reach figure ignored self-collision, which was the biggest unverified claim.

- Added collision checking to the IK study:
  - all arm geoms vs base, spine, torso, head and the other arm
  - the idle arm in a realistic tuck pose
  - explicit exclusion of the torso/spine slide pair (MuJoCo treats the fixed robot body as world, so its parent filter doesn't apply)
- **Result with collisions on:** table 95 % → **32 %**, counter 95 % → **20 %**. The only cause: the hand folding into the forearm at > 90° wrist pitch.
- **Fixes:**
  - The model's forearm bone now ends 60 mm before the wrist axis (**this is a CAD requirement**).
  - Wrist range ±140° → **±120°**. A sweep showed ±120° beats ±140° (table 50 % → 100 %) because it avoids folded solutions; ±140° was physically unrealistic anyway.
- **Added:**
  - an angled 45° counter grasp target (100 %)
  - a **two-arm IK study** (shared lift, collision-free): hold a box at the table, handover, lift a box from the floor, fold a towel → **4/4 feasible**
- **Scorecard:** reach coverage (now collision-free, 7 target sets) **93 %** vs 95 % target. It's below target only because of **top-down counter grasps at 62 %**. Tip margin min 70 mm, all other rows unchanged.
- **Considered, not taken (yet):** lift max 1.25 m gives 100 % top-down counter reach, but the spine becomes 1.26 m long (harder to carry). New queue item 1b: a two-stage telescoping spine.
- **Next:** 1b or cartridge CAD (queue #2). Cartridge CAD needs the GIM6010-8 mounting drawing first.

## 2026-11-25: loop 1b, counter reach trade-off

- Swept shoulder ranges with the same spine (0.35–1.05 … 0.55–1.25):
  - Shifting up buys top-down counter reach (62 → 100 %) but loses floor (100 → 58–75 %).
  - Only extra travel gets both.
- Extra travel = telescoping spine (~0.6 kg + cascade belts) or a 1.26 m spine (harder to carry). Not worth it now.
- **Decision D4b:** counter grasps use top-down **or** 45°, whichever is collision-free (100 %). Top-down-only stays as an info row. Scorecard reach: **93 % → 98 %** (definition change logged here: counters scored on the approach the robot will actually use).
- **Next:** cartridge CAD (queue #2): find the GIM6010-8 mounting drawing first.
