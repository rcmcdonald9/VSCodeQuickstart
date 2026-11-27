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

## 2026-11-25: loop 2, cartridge standard (KX v1)

**Picked:** queue #2. Every later CAD item depends on how a motor attaches to a bone.

- **GIM6010-8 drawing:** not obtainable here (the vendor CDN is blocked; the one direct download was refused). The design isolates it: only the motor adapter and the can's bolt circle depend on the motor. Those numbers are flagged ESTIMATE, and there's a **slotted bolt-circle coupon** to read the real pattern when a motor arrives.
- **KX v0 → v1 (reversal inside the loop).** v0 used a printed V-groove kinematic face on both sides of every cartridge. The load check killed it:

| Finding | v0 | v1 |
|---|---|---|
| Bending through the interface | Arm moment up to **11.7 N·m** at the shoulder-roll cartridge, plus belt pull (~730 N at GIM peak on a 15 mm pulley) | **0**: cartridges carry torque only. The hub turns in a 61808/61805 bearing pair (static safety ≥ 5.6), and the housing is held by a torque pin in a radial slot (shaft-mounted gearbox precedent) |
| Ball contact stress | 6 mm ball in printed PETG groove: **~200 MPa** (4× yield → dents, loses zero) | Ball on two hardened dowel pins: **3,226–3,554 MPa** (under the 4,200 MPa static limit for bearing steel) |
| Preload / twist effort | 250 N needed (β 30°) → **4.2–6.3 N·m** twist | β 20°, Ø10 balls: **160 N → 2.1–2.7 N·m**, set by a spring stack on a PTFE washer (±13 % for a ±0.2 mm print error) |
| Slip torque | – | **12.7 N·m** (1.16× GIM peak): the seat is also an overload clutch |

- **Geometry verified in CAD:**
  - Zero interference in the mated state.
  - The male face inserts freely with the collar unlocked.
  - Teeth clear the lugs for the first ~20° of twist, then ramp the collar up 1.0 mm over the last 10–15° (the preload stroke).
  - The locked collar retains the lugs.
- **Parts (`rnd/parts/`, STLs in `rnd/out/`):**
  - KX74 / KX44: male adapter, hub with pin bores, collar, retainer
  - cartridge M can with torque pin
  - cartridge S sleeve for the STS3215 (horn adapter 4×M3 / 9.9 mm square)
  - GIM6010 bolt-circle coupon
- **Print times** (PrusaSlicer generic profile, 0.2 mm, PETG; the P2S should be ~3× faster, ESTIMATE):
  - KX74 interface set (hub + collar + adapter + retainer): **8.5 h**
  - cartridge M can: 4.4 h
  - KX44 set: 3.9 h
  - cartridge S sleeve: 1.5 h
- **Analysis:** `rnd/kx_check.py` → [rnd/kx.md](rnd/kx.md). New scorecard rows: KX slip/peak **1.16×** ✅, socket bearing safety **5.6** ✅. Cost **$2,800 → $2,900** (+$100 KX hardware: bearings, pins, balls, springs, ESTIMATE).
- **Housekeeping:** `__pycache__` removed from git; `.gitignore` added.
- **Open:**
  - the shaft-mounted principle moves belt loads onto separately supported pulley hubs (arm CAD must do this)
  - the spring-stack part needs a real supplier
  - a clutch slip of more than 120° needs a re-home (magnet index)
- **Next:** arm CAD v0 (queue #3) with KX sockets; 2b follow-ups as they become blocking.

## 2026-11-25: loop 3, belt reductions + elbow stage (arm CAD begins)

**Picked:** queue #3, arm CAD. It's the next build-enabling step, and the biggest new risk from loop 2 was belt pull: ~690 N on a small pulley at GIM peak.

- **Belt sizing** (`rnd/belts.py` → [rnd/belts.md](rnd/belts.md)):
  - HTD 5M, 15 mm, 20T small pulleys.
  - Pull at actuator peak is 691 N against 975 N allowable → **1.41×**.
  - 7.2–8.5 teeth in mesh (≥ 6 ✅).
  - Pulley pitch diameters: 31.8 / 63.7–95.5 mm.
  - Belt ratings: Gates manuals are unreachable from this environment, so I used secondary vendor tables (650 N per 10 mm allowable). Flagged ESTIMATE in `components.yaml`.
  - Precedent: Berkeley Blue (single-stage belt reductions, >95 % efficiency).
- **Idea tested and rejected: remote elbow motor.** Moving the elbow motor 0.14 m up the upper arm on a long belt (Blue-style):

| | At the elbow (kept) | Remote at 0.16 m |
|---|---|---|
| Shoulder roll margin, spec / worst | 1.70× / 1.08× | 1.72× / 1.09× |
| Upper module mass | 9.47 kg | 9.71 kg |

  The 85 % shoulder spring already removes most self-weight torque, so the gain is tiny, and the extra pulley hubs cost carry mass. The option stays as a parameter (`arm.elbow_motor_at`) for future studies.
- **CAD (`rnd/parts/belt_stage.py`):**
  - printed HTD 5M pulley generator (20/40/50/60T STLs)
  - small-pulley hub: KX74 face + Ø20 journals, the pulley straddled by two **61804** bearings (changed from 6004 to save 10 mm)
  - elbow stage layout render: [img/elbow_belt_stage.png](rnd/img/elbow_belt_stage.png)
- **New finding:** a GIM-driven joint is **~100 mm wide** along its axis (cartridge 56 mm + pulley hub 44 mm). The twin's capsule arm is thinner, so collision-free reach may be slightly optimistic. The twin v1 must use the real widths.
- **Scorecard:** new row "belt safety at peak" **1.41×** ✅. All other rows unchanged (reach 98 %, upper 9.5 kg, cost $2,900).
- **Next:** bones around the sockets, with CAD masses replacing estimates; then the shoulder module + spring.

## 2026-11-25: loop 4, arm bones from CAD; the mass budget breaks and is re-split

**Picked:** arm bones (queue #3). The bone masses were the largest unverified ESTIMATE behind R3 (carry) and R2 (joint margins).

- **CAD (`rnd/parts/arm.py`):**
  - **Upper bone:** Ø72 neck tube into an elbow housing. The housing holds the elbow cartridge 70 mm above the elbow axis, its pulley hub, the belt and the 40T pulley hub on 2 × 61808 bearings.
  - **Forearm:** a side link. The wrist cartridge's output faces back toward the arm's centre plane, so the hand has ~0 lateral offset and swings beside the link rather than into it.
  - Arm assembly render: [img/arm_v0.png](rnd/img/arm_v0.png). This is a functional layout, not the industrial design.
- **Mass from CAD vs estimate (per arm structure):**

| | Estimate | CAD v0 (2.4 mm walls, KX74 everywhere) | CAD v0.1 (1.6 mm walls, KX56 for DM-J4310, hollow elbow shaft) |
|---|---|---|---|
| Bones + sockets + hubs + bearings | 0.45 kg | 1.14 kg | **0.93 kg** |

  The estimate left out the sockets, hubs and bearings (0.38 kg per arm). The shoulder stages were never counted either; they're now an ESTIMATE of 0.30 kg per stage plus 0.15 kg of spring per arm.
- **New interface size KX56** (DM-J4310): slip 8.3 N·m (1.19× peak), twist 1.4–1.8 N·m, collar Ø78 instead of Ø98.
- **Scorecard, before → after:**

| Metric | Before | After |
|---|---|---|
| Upper module | 9.5 kg ✅ | 11.9 kg assembled ❌ → **re-split: core 6.2 kg ✅ + 2 arms of 2.8 kg ✅** |
| Total robot | 16.8 kg | **19.2 kg** |
| Joint margin min (rated) | 1.70× | **1.57×** ✅ |
| Tip margin | 70 mm | **72 mm** ✅ (heavier torso) |
| Socket bearing safety | 5.6 | **3.4** ✅ (yaw socket moved to the smaller 61805 pair) |
| 15 mm threshold torque | 3.7 N·m (stale hard-coded value) | **4.2 N·m** of 5 rated ✅ (now computed from mass) |

- **Decision D1 changed:** arms detach at the shoulder-pitch KX. It's the same twist-collar as every joint, plus a blind-mate plug (new queue 6b). That also serves the owner's "swap arms and hands over time" goal.
- **Thin margins to watch:** base 5.8/6 kg; threshold torque 4.2/5 N·m (heavier robot); joint margin 1.57/1.5×.
- **Next:** shoulder module CAD (it replaces the last big mass ESTIMATE); arm mass diet.

## 2026-11-25: loop 5, the "85 % spring" assumption tested; workspace torque check finds an under-rated joint

**Picked:** the shoulder margin rested on an unverified claim: a spring cancelling 85 % of the arm's self-weight in every pose. A pitch-only spring can't do that once the elbow bends, so I tested it before drawing the shoulder.

- **Balancer study** (`rnd/spring.py` → [rnd/spring.md](rnd/spring.md)):
  - 6,000 random poses in the MuJoCo twin, static torques.
  - A zero-free-length cable-spring balancer (Anglepoise / Herder principle) gives τ = K·sin(pitch).
  - The real cancellation is **~36 %** at the 95th percentile, not 85 %.
  - A K sweep picked **K = 4 N·m** (best for a 0.5 kg typical payload). The spring has a preload screw so it can be re-tuned.
  - Shoulder pitch still holds: **1.61×** rated vs 95th-pct need, **2.57×** peak vs worst pose.
- **New whole-workspace check for every joint.** The old check only looked at a straight horizontal arm, which never loads the upper-arm yaw. Result:

| Joint | Before | After |
|---|---|---|
| Upper-arm yaw | DM-J4310 direct: need 4.6 N·m (p95) vs 3 rated → **0.65× ❌** | **GIM6010-8 × 2 belt → 2.06×** ✅ |
| Shoulder roll | GIM × 3: **1.29× ⚠️** | **GIM × 3.5 (20T:70T) → 1.51×** ✅ |
| Shoulder pitch / elbow / wrist pitch | – | 1.61× / 1.77× / 2.15× ✅ |

- **Why GIM × 2 for the yaw** (over a DM-J4340P direct, 1.96×): it's ~$70 cheaper per arm, backdrivable (40:1 gearing isn't), and the same part as the other big joints. The cost is +0.29 kg per arm.
- **Scorecard, before → after:**
  - new rows: workspace margin **1.51×** rated / **2.57×** peak ✅
  - cost **$2,900 → $2,761** (actuators $1,550 → $1,411)
  - arm piece 2.8 → **3.1 kg**; assembled upper 11.9 → 12.5 kg (info)
  - socket bearing safety 3.4 → 5.1
  - threshold torque 4.2 → **4.3** of 5 N·m (watch)
  - reach / tip unchanged
- **Lesson recorded:** size joints on the workspace, not on a single "worst" pose that someone picked.
- **Next:** shoulder module CAD with the cable-spring balancer; arm mass diet (the arm piece is now 3.1 kg).

## 2026-11-25: loop 6, the reach claim re-checked against real CAD geometry

**Picked:** the headline reach number (98 %) came from thin capsules. The CAD arm is much bulkier (~110 mm elbow, side-link forearm, protruding elbow cartridge), so this was the biggest unverified claim.

- **Twin collision geometry from CAD** (`rnd/parts/collision.py`):
  - five convex hulls per arm (neck, elbow housing, protruding elbow cartridge, forearm link, wrist cartridge + collar), mirrored for the right arm
  - loaded as MuJoCo mesh geoms; masses unchanged
  - `twin.GEOMETRY = "cad"` is now the default (capsules remain as an option)

| Reach (collision-free) | Capsules | CAD hulls | CAD hulls + idle-arm re-park |
|---|---|---|---|
| Mean (scored sets) | 98.2 % | **92.5 % ❌** | **98.2 % ✅** |
| Floor pickup | 100 % | 80 % | 100 % |
| Dishwasher rack | 94 % | 80 % | 94 % |
| Counter top-down (info) | 62 % | 60 % | 60 % |

- **Cause of the drop:** not the working arm. The *idle* arm, parked hanging down, put its bulky elbow housing into the base when the torso was lowered for floor work. **Fix:** the idle arm may re-park in one of 4 poses (hanging, folded forward 35° or 70°, pitched back 40°), whichever is collision-free. This is a planner rule, now in the twin.
- **Tip margin** now uses the park pose actually chosen: **72 → 68 mm** (≥ 50 ✅).
- **Bimanual 4/4** still feasible with CAD geometry.
- **Open:**
  - The shoulder, torso and base are still primitives. Their CAD will need the same check.
  - Twin renders still show the default park pose.
- **Next:** shoulder module CAD (pitch + roll stages, cable-spring balancer, detach plug).

## 2026-11-25: loop 7, shoulder pitch module: real balancer + V-band arm detach

**Picked:** shoulder module CAD (queue #3). It held two unresolved designs: the balancer mechanism, and how a detachable arm carries its bending moment.

- **Balancer** (`rnd/balancer.py` → [rnd/balancer.md](rnd/balancer.md)):
  - Zero-free-length cable-spring (Anglepoise / Herder / French & Widden): pulley A sits above the axis, pin B on a lever on the pitch shaft, and the cable runs back into a caged compression spring.
  - Exact τ = k·a·b·sin(pitch). The Ø12 pulley adds ≤ 0.48 N·m of error, small next to 11.9 N·m rated.
  - CAD forced b from 40 to **30 mm** so pin B clears pulley A when the arm points straight up (|a − b| ≥ 15 mm).
  - Result: spring **2.7 N/mm**, 61 mm stroke, 213 N max. K is adjustable **3.6–5.2 N·m** by sliding pulley A.
  - If the cable breaks, the motor alone still holds the arm.
- **Arm-detach coupling KX-V** (in [kx.md](rnd/kx.md)):
  - The detach plane carries the arm's **full bending** (19 N·m with ×1.5 dynamic), which the hand-twist collar can't preload for.
  - Switched to a **V-band clamp**: printed 3" sanitary flanges so a stock stainless quick-release clamp fits, 3 dowels, face contact. Precedent: turbo V-band, Marman clamps, tri-clamp fittings.
  - Lift-off **40 N·m at 1 kN preload → 2.1×** ✅. Flange pressure 0.9 MPa.
- **Shoulder pitch module CAD** (`rnd/parts/shoulder.py`):
  - Contents: bracket, pitch shaft (50T pulley without flanges, balancer lever, KX-V ferrule), small-pulley hub + KX74 socket, GIM cartridge, pulley A on a slotted post, horizontal spring cage.
  - Interference-checked; one 0.07 cm³ touch at a bearing seat remains.
  - Render: [img/shoulder_pitch_v0.png](rnd/img/shoulder_pitch_v0.png)
- **Masses:** pitch module **0.81 kg from CAD** (estimate was 0.45). Roll stage re-estimated at 0.60 kg (was 0.30) by scaling from this CAD.

| Scorecard | Before | After |
|---|---|---|
| Upper core / each arm | 6.2 / 3.1 kg | **6.9 / 3.4 kg** ✅ |
| Assembled upper (info) | 12.5 kg | 13.8 kg |
| 15 mm threshold torque | 4.3 N·m | **4.5 of 5 N·m** ⚠️ trend |
| Tip margin | 68 mm | 72 mm |
| Arm-detach lift-off | – | **2.1×** ✅ |

- **Critique:** every CAD pass has roughly doubled an estimate. The assembled upper went 9.5 → 13.8 kg across loops 4–7. Carry pieces still pass, but base threshold torque is now the margin to watch. **Next loop: a mass diet**, with base-drive fallbacks ready (8" wheels or a belt gear-down).

## 2026-11-25: loop 8, lift decided (GIM + ratchet pawl), first mass-diet pass

**Picked:** the lift. A mass ranking showed the lift motor was still an un-designed 0.9 kg placeholder (NEMA23 + brake), and the power-off torso drop was an unretired safety risk. That beat shaving plastic.

- **Lift study** (`rnd/lift.py` → [rnd/lift.md](rnd/lift.md)): moving mass 12.2 kg + 2 kg payload. Four options compared:

| Option | Mass | Power-off | Verdict |
|---|---|---|---|
| A: NEMA23 + brake | 1.25 kg | brake | heaviest; another motor type |
| B: constant-force springs + NEMA17 | 0.60 kg | drifts, needs a brake | **rejected:** shoots up when the arms are removed for carrying; spring fatigue |
| **C: GIM6010-8 + HTD 5M + spring-applied ratchet pawl** | **0.55 kg** | **pawl drops in (fail-safe), 0 W holding** | **chosen**: the standard reusable cartridge |
| D: self-locking lead screw | 0.70 kg | locks | 23 s full travel, too slow |

  Option C numbers: torque margin **2.10×**, full travel **3.5 s**, belt 6.5×, pawl tooth shear 9 MPa at ×3 catch shock.
- **Shoulder bracket lightening:** pockets keep ≥ 10 mm of material around the bearing seats. Pitch module 0.81 → **0.76 kg**; interference re-checked.
- **Diet lever ruled out:** the 6" DuraOmni (0.45 kg) is already among the lightest 6" omni wheels (alternatives 0.6–1.3 kg).

| Scorecard | Before | After |
|---|---|---|
| Upper core | 6.9 kg | **6.5 kg** |
| Assembled upper (info) | 13.8 kg | **13.3 kg** |
| Cost | $2,761 | $2,764 (+1 GIM, −NEMA kit) |
| Lift (new row) | – | 2.10× / 3.5 s ✅ |
| Tip margin | 72 mm | 70 mm |

- **Next:** arm-bone diet (1.8 kg for both arms: ribbed 1.2 mm shells, or aluminium tube bones as in Blue), then base CAD (the chassis 1.4 kg is still an ESTIMATE).

## 2026-11-25: loop 9, bone strength and stiffness; carbon tubes (owner moved the loop to hourly)

**Owner:** shortened the loop interval (now hourly at :10) and asked for this run immediately.

**Picked:** the arm diet. Before thinning walls, I checked what limits the bones. Nothing had checked their strength or stiffness yet.

- **New analysis** `rnd/bones.py` → [rnd/bones.md](rnd/bones.md): arm straight out with 1 kg, ×1.5 dynamic; printed PETG E 1.8 GPa, 12 MPa allowable (layer adhesion, SF 2).
  - **Strength is never binding:** ≤ 4 MPa even at 1.2 mm walls.
  - **Stiffness is.** Hand sag from bone bending alone: 2.9 mm (2.4 mm walls), **4.1 mm (1.6 mm, the current design)**, 5.2 mm (1.2 mm).
- **Decision D4d:** carbon tubes for the two slender sections (neck Ø40×1, forearm Ø30×1) in printed sockets, with 1.2 mm shells elsewhere → **sag 1.6 mm** ✅ (target ≤ 2). Precedent: Hello Robot Stretch's carbon arm, Kinova carbon links. Cost +$25 (ESTIMATE).
- **CAD updated** (`rnd/parts/arm.py`): tube sockets, `cf_neck()`, `cf_fore()`. Arm structure **0.93 → 0.89 kg** per arm. This includes correcting the yaw socket to KX74, which was missed when the yaw became a GIM in loop 5 (+0.03 kg).
- **Honest conclusion on the diet:** the bones were already thin. Each arm piece (3.4 kg) is ~60 % actuators + belt stages, so the 2.5 kg/arm target is dropped. Carry passes anyway (core 6.5 kg, arm 3.4 kg).
- **Scorecard:** new row "hand sag" **1.6 mm** ✅. Cost $2,764 → $2,788. Tip margin 70 mm; all other rows unchanged.
- **Next:** base CAD (the chassis 1.4 kg and base layout are still ESTIMATEs; the base is the last major untouched module).

## 2026-11-25: owner input, head fixed on the mast top (D8)

The owner asked why the head doesn't stay at the top. Evaluated and adopted:

- **Twin:** the head is now a fixed body at the mast top (centre 1.225 m; its underside clears the torso at full lift by 30 mm).
  - Reach still **98 %**, bimanual 4/4, tip margin min **70 mm** (unchanged).
  - Arms vs head collisions are checked.
- **Lift:** the 0.6 kg head leaves the moving mass → torque margin **2.13 → 2.22×**.
- **Head camera check:** new head cameras in the twin; views in [img/head_camera_views.png](rnd/img/head_camera_views.png).
  - Tilted 60° down, the head sees the hand at the floor and on the table.
  - Level/up, it sees the shelf hand.
  - The torso blocks the lower part of the frame during floor work; the wrist cameras cover close-up work (Stretch precedent).
- **Looks:** the overall height is a constant 1.29 m, and the mast no longer pokes through the head. The new problem is that the mast now reads as a long neck when the torso is low: give it a shaped cover with a light strip.
- Renders at low and high torso: [img/robot_head_on_mast.png](rnd/img/robot_head_on_mast.png).

## 2026-11-25: build-out session (owner: finish the full design + simulation, then resume the loop). Step 1: wrist, hand, gripper

**Owner direction:** pause the hourly loop until the whole robot is designed, simulated and working. Trigger disabled. Milestone criteria:
- every module in CAD
- a full interference- and print-checked assembly with CAD masses
- a dynamic simulation of driving + pick-and-place within actuator limits
- the electrical plan
- BOM, print plan and assembly guide

**Hand CAD** (`rnd/parts/hand.py`):
- DM-J4310 cartridge D (KX56; bolt circles ESTIMATE)
- palm holding the wrist-roll cartridge S
- **SO-100-style gripper**: a fixed jaw + a jaw on the horn of the gripper servo, which lies on its side under the base plate (fewest parts; LeRobot precedent)
- wrist camera
- the forearm wrist can re-sized around the real cartridge; the hand centre plane sits on the arm's centre plane

| Finding | Before | After |
|---|---|---|
| Wrist-pitch axis → grasp point (`l_hand`) | 0.13 m (assumed) | **0.15 m** (real stack: wrist roll + KX44 + gripper servo) |
| Hand mass (`gripper_mech_mass`) | 0.10 kg ESTIMATE | **0.31 kg** from CAD |
| Workspace joint margin | 1.51× | 1.38× ❌ (roll), pitch 1.42× → fixed: **pitch 2.5 → 3:1 (20T:60T), roll 3.5 → 4:1 (18T:72T), K 4.0 → 4.5 N·m → 1.54×** ✅ |
| Belt safety (18T roll pulley) | 1.41× | **1.27×** ✅ |
| Reach (with CAD hand hulls in the twin) | 98 % | **98 %**; bimanual 4/4 |
| Arm piece / assembled upper | 3.4 / 13.3 kg | 3.6 / 13.7 kg |
| Tip margin | 70 mm | 67 mm |

- Interference: shoulder module clean (60T pulley); arm assembly clean. The moving jaw brushes the gripper-servo sleeve by < 1 cm³ at 25° (v0, to refine).

## Build-out step 2: arm-side shoulder (roll + yaw stages) and V-band orientation

- **`rnd/parts/roll.py`:**
  - male KX-V ferrule
  - roll link (walls at the small-hub journals)
  - roll shaft + 72T pulley on 2 × 61808
  - roll cartridge behind
  - clevis arms clamped to the shaft ends
  - yaw block (two plates) with the yaw shaft + 40T pulley + upper-bone flange
  - yaw cartridge (belt centre 80 mm)
- Interference: clean apart from sub-cm³ touches at bearing seats.
- **Kinematics changed by the CAD:** the Ø114 roll pulley doesn't fit the twin's offsets.
  - Roll axis 50 → **75 mm** outboard of the detach plane; yaw interface 70 → **100 mm** below the roll axis.
  - The upper bone shortens (ELBOW_Z 230 → 200 mm), so l_upper stays 0.30 m.
  - Twin, sizing, bones and socket analyses all read the new `arm.roll_offset_y` / `arm.yaw_drop`.
- **V-band flanges fixed:** flat faces meet at the detach plane (pitch-body y = 0), tapered backs face away from each other, spigots point into their own modules.
- **Twin collision now includes the shoulders:** roll link + roll cartridge (pitch body), yaw block (roll body), torso-side pitch modules (torso body).
- **Masses from CAD:** roll stage 0.58 kg, yaw stage 0.47 kg (estimates were 0.60 / 0.20).

| Scorecard | Before | After |
|---|---|---|
| Reach | 98 % | **99 %** (wider shoulders) |
| Tip margin | 67 mm | **61 mm** ✅ |
| Workspace joint margin | 1.54× | **1.51×** ✅ |
| Each arm / assembled upper | 3.6 / 13.8 kg | **3.9 / 14.3 kg** |
| Lift margin | 2.14× | 2.07× |
| 15 mm threshold | 4.5 | **4.6 of 5 N·m** (watch) |

## Build-out step 3: mast, lift, carriage, torso, head

- **Mast sizing** (`rnd/mast.py`): worst static moment at the mast foot is 46 N·m (both arms straight out with 1 kg, torso at the top).

| Mast | Torso sway | kg/m |
|---|---|---|
| 2040 extrusion (v0 placeholder) | 5.5 mm ❌ | 0.50 |
| 50×50×2 Al square tube | 2.6 mm | 1.04 |
| **60×60×2 Al square tube (chosen)** | **1.4 mm** ✅ | 1.25 |

- **CAD** (`rnd/parts/upper.py`):
  - **Lift:** GIM cartridge at the mast foot with a 20T pulley; HTD 5M belt to an idler in the head bracket; printed ratchet strip on the mast's back face.
  - **Carriage:** printed sleeve with 8 × 625 bearings, a slot over the ratchet, and the pawl + solenoid.
  - **Torso:** two-piece shell with side windows for the pitch modules; electronics tray (Pi 5, DC-DC), e-stop, chest light.
  - **Head:** top bracket with idler and carry handle → pan cartridge S → yoke → tilt cartridge S → shell with a dark visor, eye lights and the RGB-D camera.
  - Interference clean, apart from mounting interfaces (belt slot, servo horn).
- **Masses from CAD:**

| Item | Estimate | CAD |
|---|---|---|
| Carriage | 0.35 kg | 0.58 kg |
| Torso | 0.90 kg | 0.53 kg |
| Head | 0.60 kg | 0.88 kg |
| Lift drive | 0.55 kg | 0.81 kg |
| Mast | 0.85 kg | 1.33 kg |

  Upper core 6.5 → **7.3 kg** ✅; assembled upper 15.1 kg.
- **Head height** follows the CAD: the head centre sits 118 mm above the mast top, so the **overall height is 1.39 m** (was 1.29 m).
- **Scorecard:** tip margin 61 → 65 mm; lift 2.09×; new row "torso sway 1.4 mm" ✅. **15 mm threshold 4.8 of 5 N·m** (tight; addressed in the base step).

## Build-out step 4: base

- **CAD** (`rnd/parts/base.py`):
  - 4 wheel modules ("+" layout, one wheel forward, axles radial), each: GIM cartridge pointing inward → KX74 wheel hub → Ø40 shaft in 2 × 61808 in a two-plate pod → DuraOmni flange → 6" omni wheel. Wheel loads never reach the motor bearing.
  - The four cartridges leave a centre bay for the 6S battery, which slides out from the back.
  - Ribbed deck and skirt, each in four printable quarters.
  - Mast coupling with a cam lever and a blind-mate plug pass-through.
  - Lidar on the front edge, bumper light ring, base electronics in a diagonal bay.
  - Interference: **none**.
- **Masses from CAD:** structure 1.4 → **2.0 kg**, shell 0.45 → 0.33 kg. **Base 5.8 → 6.3 kg.**
  - The internal 6 kg allocation is raised to **8 kg**. The real requirement (R3) is ≤ 10 kg per carried piece and 2 trips; the trips are core 7.3 + base 6.3, then battery 1.5 + arms 2 × 3.9.
- **R4 metric redefined:** climbing a 15 mm threshold is a sub-second transient, so it is now checked against the GIM's **peak** torque: **2.3×** (4.9 of 11 N·m). Continuous carpet driving uses 0.8 N·m of 5 rated.
- Tip margin 67 mm.

## Build-out step 5: whole-robot assembly check

`rnd/assembly_check.py` → [rnd/assembly.md](rnd/assembly.md):

- **Printability:** every printed piece tested in its best orientation on the P2S bed (256 mm cube).
  - First pass: 53/55. The **ratchet strip** (830 mm) is now 4 bonded segments, and the **torso shell** (300 mm) is left/right halves.
  - Now **59/59 fit** ✅.
- **Interference between modules** at lift 0.40 / 0.75 / 1.05 m (arms reaching forward at the bottom, render pose otherwise). First pass found three real clashes:
  - the lift-motor mount and the base mast coupling occupied the same space → the mount now sits on top of the coupling
  - the torso electronics tray hit the pitch cartridges that reach into the torso → the electronics moved forward
  - the elbow hit the base at 0.40 m lift in the render pose (not a real working pose; the twin already guarantees collision-free reach poses)
  - Now **0 clashes** ✅.
- **Mass roll-up:**

| Piece | kg |
|---|---|
| Base | 6.31 |
| Upper core | 7.31 |
| Each arm | 3.87 |
| Battery | 1.50 |
| **Total robot** | **22.9** |

- New scorecard rows (cached from the last `python -m rnd.assembly_check`): R13 printability 59/59 ✅, clashes 0 ✅.

## Build-out step 6: twin v1, dynamic fetch task within the real actuator limits

`rnd/sim.py` → [rnd/sim.md](rnd/sim.md), [video](rnd/img/sim_fetch.mp4), [frames](rnd/img/sim_fetch_frames.png).

**Model:**
- MuJoCo, derived from the kinematic twin (CAD collision hulls).
- Holonomic base force-limited to the four GIM omni wheels.
- Every joint a position servo capped at its real peak torque (motor × belt × efficiency); lift capped by the GIM on a 20T pulley.
- Coupled two-finger gripper (μ 1.0, 50 N squeeze).
- 0.35 kg can on the floor; 0.75 m table that the robot must not touch.

**Task:** drive 0.4 m → reach → descend → grasp → lift → carry → drive 0.7 m → place on the table → release → retract → back away → tuck.

**Debugging (each failure was real, not tuned away):**

| # | Failure | Cause | Fix |
|---|---|---|---|
| 1 | Can knocked over at the pick | Gripper command inverted (the sim's own bug) | Fixed the command |
| 2 | Can slid out on the way down | Can slipped 25 mm in the hand during the carry | Squeeze at full rated force; place height from the measured in-hand offset (wrist-camera stand-in) |
| 3 | Can dragged after release | Uncoupled fingers let the can's weight push the pair sideways | Coupled fingers (rack-and-pinion behaviour) |
| 4 | Idle arm through the table | The table didn't collide with the robot | Table collides; the idle arm parks pitched back; contacts are counted |
| 5 | Can knocked over at retract | Wrist spun on retract and the fingers dragged the can | IK continuity term; fingers open to 120 mm; back away before tucking |
| 6 | Can drifted 30 mm in the grip | **The joint-space carry swung the hand through 80° of tilt.** A planning lesson for the AI phase: carry moves must be Cartesian with the orientation held | Cartesian carry with the hand held down |

**Result:**
- **Can placed upright on the table, 0 robot–table contacts.**
- Peak actuator use ≤ 33 % of peak (shoulder pitch, gripper).
- Continuous use ≤ 37 % of rated, except the gripper squeeze at 61 %.
- Min dynamic tip margin **78 mm**.
- **Robustness:** ±15 mm perception error × 0.2–0.6 kg → **8/8 succeed** open-loop and **8/8** with the wrist-camera re-measure (σ 3 mm, ESTIMATE).
- Scorecard rows added (cached from the last `python -m rnd.sim`).

## Build-out step 7: electrical, BOM, print plan, build guide → milestone M2

- **Electrical** (`rnd/electrical.py` → [rnd/electrical.md](rnd/electrical.md)):
  - 40 A main fuse; e-stop contactor on the motor bus while logic stays up; branches 20/15/15/10 A plus 12 V and 5 V DC-DC.
  - 3 CAN buses at ~65 % load (500 Hz) and 3 TTL buses.
  - Detach and coupling plugs: XT30/XT60 + pogo blocks.
  - Safe-stop-1 e-stop behaviour.
  - **New finding:** the pack's BMS must handle **≥ 30 A continuous, ~68 A for 1 s**. Common 10–15 A 6S BMS boards would trip.
- **Print plan** (`rnd/print_plan.py`, every part sliced with PrusaSlicer): **10.4 kg PETG** (~$210), 730 h generic ≈ **255 h on the P2S** (ESTIMATE factor 0.35).
- **BOM** (`rnd/bom.py`, quantities from CAD): **$3,220** ✅ (target $3,300).
  - Actuators $1,494; compute + sensors $694; wheels + power $412; printed parts $250; drive + structure $201; bearings $104; KX hardware $66.
  - This replaces the parametric cost estimate ($2,788) as the scorecard's cost row.
- **Build guide** ([BUILD_v4.md](BUILD_v4.md)): about **25 h hands-on** for the first build → **R10 (≤ 4 h) is not met**. It's the one failing metric and the top item in the next queue.
- **Milestone M2 reached:** every module in CAD, assembly clean, fully printable, dynamic task working within actuator limits, build package written. The hourly loop resumes with the new queue (RND_PLAN §4).

## 2026-11-25: loop 10, assembly time (R10, the one failing metric)

**Picked:** R10 (first build 25 h vs 4 h). Over half was repetitive: 19 KX sockets, 21 cartridges, 13 belt stages.

| Change | Precedent | Effect (ESTIMATE) |
|---|---|---|
| **Print-in-place KX socket**: hub + retainer fused, collar printed around it (≥ 0.3 mm clearances). Pause 1 drops in the 6 dowel pins; pause 2 lays the wave spring + PTFE washer on the lip | embed-at-pause inserts; print-in-place bearings/hinges | 10 → 4 min per socket; 3 screws and one part fewer per socket |
| Cartridges: 3 of 6 housing screws, 4 of 8 output screws | the torque pin carries the reaction; 11 N·m peak → 160 N shear per M3 | 10 → 6 min per cartridge |
| **Eccentric bushings** around each small-hub bearing (±1 mm centre distance) instead of fixed centres or slots | bicycle eccentric bottom brackets; 3D-printer belt tensioners | 15 → 9 min per stage |
| Pre-made harness segments per module | – | wiring 3 → 2 h |

| Scorecard | Before | After |
|---|---|---|
| R10 first build | 25 h | **19 h** ❌ (one arm ≈ 5 h) |
| Printed pieces | 59 | **57** (retainers gone, eccentric bushings added), all fit |
| Clashes | 0 | 0 |
| BOM | $3,220 | $3,221 |
| Filament | 10.4 kg | 10.4 kg |

- **Owner decision (non-blocking):** R10 "≤ 4 h" can't be met by a 21-actuator robot. A 6-servo LeRobot arm alone takes a few hours. Proposed split: one arm ≤ 4 h, whole robot ≤ 15 h, module swap ≤ 5 min.
- **New coupon items:** the print-in-place socket (clearances, the bridge over the spring) and the eccentric bushing.

## 2026-11-25: looks pass v1 (owner request: round parts, new render)

- **New helper** `rnd/parts/soft.py`: boxes and shells with every edge rounded.
- **Torso:** now a rounded body (230 × 390 × 210 mm, two printable halves) that **encloses both shoulder pitch modules**. The arms emerge through round ports.
  - The first size (204 × 370 × 194) let the pitch-bracket corners poke through the rounded edges. The whole-robot check caught it, so the torso grew.
  - Assembly check: 0 clashes, 57/57 printable.
- **Head:** pill-shaped, with a rounded visor.
- **Elbow housings:** soft on every edge.
- **New covers:**
  - a **shoulder cap** over the roll cartridge and link top (open underneath; it clears everything that rolls)
  - a **rounded band** around the upper-arm twist (yaw) block
- **Twin:** the torso collision box grows to match, and the cap is added to the shoulder hulls.
  - Reach stays **99 %**, bimanual **4/4**, tip margin 66 mm, workspace joint margin 1.51×.
  - Upper core 7.3 → 7.6 kg (bigger torso shell). Head 0.88 → 0.82 kg.
- **Render:** [img/robot_rounded_v1.png](rnd/img/robot_rounded_v1.png).
- **Still to do for looks:**
  - the bare mast reads as a long neck (cover + light strip)
  - the lift motor at the mast foot is exposed
  - the wrist/hand block is still boxy
  - the base deck needs a rim

## 2026-11-25: loop 11, looks pass v2

- **Mast:** black anodised. A cover isn't possible because the carriage's 625 bearings run on all four faces. Dark recedes visually, so the "long neck" reads as a spine and the white body and head carry the form (render check).
- **Lift drive:** rounded cover at the mast foot, open toward the mast, with a belt slot.
- **Hand:** palm and gripper base are soft on every edge.
- **Base:** rounded rim on the deck edge (printed in arcs).
- **Checks:**
  - local interference clean (sub-cm³ mounting touches)
  - whole-robot check: 0 clashes, all pieces printable
  - twin unchanged: reach 99 %, tip margin ≥ 50 mm
- **Render:** [img/robot_rounded_v2.png](rnd/img/robot_rounded_v2.png).
- **Still boxy / next looks items:**
  - the yaw-block bands
  - the exposed orange collars at the shoulder roll (they read as "expressed joints" and are kept on purpose)
  - the wheel windows in the skirt (add fender lips)

## 2026-11-25: loop 12, omni base over thresholds and carpet (roller-level simulation)

**Picked:** the oldest unretired physical risk. Omni-wheel threshold climbing and carpet driving had only hand calculations (R4, flagged since loop 0).

- **New model** (`rnd/threshold.py` → [rnd/threshold.md](rnd/threshold.md)):
  - the robot as one rigid body (25.6 kg incl. 2 kg payload, COM 0.40 m from the twin)
  - four 6" omni wheels in the "+" layout, each with 2 × 10 free rollers
  - wheel velocity servos torque-limited at the GIM peak
  - hard floor vs carpet-like ground (soft contact + rolling friction, c_rr ≈ 0.05)
  - sharp steps of 0–20 mm
- **Two modelling bugs found and fixed:** wheel-direction sign; rollers colliding with each other.

| Case (0.30 m/s) | Result | Peak wheel torque |
|---|---|---|
| 15 mm, 45°, hard / carpet | ✅ / ✅ | 9.9 / 10.3 N·m (of 11) |
| 15 mm, **0° (straight at a wheel)** | **❌** | sideways rollers can't climb: confirms the 45° rule |
| 20 mm, 45°, carpet | ✅ | 11.0 N·m (at the limit) |
| **15 mm / 20 mm at 45° and 0.15 m/s** | ✅ | **5.6–7.1 N·m → 1.5–2.6× margin** |
| Carpet cruise 0.3 m/s | ✅ | 2.2 N·m RMS per wheel (44 % of rated; hand estimate was 0.8) |

- **Decision D2b updated:** cross thresholds at 45° **and ≤ 0.15 m/s**. The static check (4.9 N·m) under-predicted the impact peaks by 2×; the planner rule restores the margin.
- **Scorecard rows added:** threshold sim **crossed, 1.7×** ✅; carpet cruise **2.2 / 5 N·m** ✅.
- **Still to retire with hardware:** a real carpet and threshold bench test. Roller geometry and carpet model are ESTIMATEs.

## 2026-11-25: loop 13, research watch: prices, ratings, battery (3 parallel research agents)

The blocked hosts (vendor CDNs, Gates, SKF) limited the agents to search snippets; every figure carries its source URL in `bom.py` / `components.yaml`.

- **Belts (important):** the 650 N/10 mm figure used since loop 3 is not a working rating. Mädler gives **312 N admissible pull** for neoprene HTD 5M-15 (PU steel-cord: 1268 N).
  - Re-rated the belt check properly: sustained pull (95th-percentile workspace holding torque) vs working rating, and peak vs ultimate/3.
  - Result: **1.4× / 5.1×** ✅ (roll is the tightest).
  - Fallback if the coupons show tooth jump: PU steel-cord belts at the same geometry.
- **Bearings:** SKF 61808 C0 = 3.65 kN (was 4.15 assumed). Generic 61808/61804/61805 cost **$4.7–5.7** each (was $1.3–2.5).
- **Battery** (new requirement ≥ 30 A BMS):
  - recommended: Lumenier NAV-class 6S 10 Ah 21700 pack (975 g, 35 A cont., $150) + Heltec 6S 30 A BMS (100 A trip) inline + ToolKitRC C6 charger
  - **Warning:** many "24 V 10 Ah" Amazon packs are 7S (29.4 V), above the GIM's range
  - battery mass 1.5 → **1.1 kg**
- **Other prices:**
  - tri-clamp quick-release $25 (flange OD 90.9 mm confirmed)
  - CF tubes ~$70
  - closed belts $8–28
  - wave springs $17 each → switched to 6 coil springs per socket
- **Motor drawings:** the search snippets of the GIM6010-8 drawing show 8-M3 / 6-M3 patterns and Ø57 / Ø34 circles, consistent with the ESTIMATE bolt circles (57 / 34). DM-J4310: Ø56–57 × 46 confirmed; hole patterns not found (the drawing is image-only).

| Scorecard | Before | After |
|---|---|---|
| **R8 BOM** | $3,221 ✅ | **$3,645 ❌** (target $3,300, limit $5,000) |
| Battery mass | 1.5 kg | 1.1 kg |
| Belts (new definition) | 1.27× peak | 1.4× sustained / 5.1× ultimate ✅ |
| Tip margin | 66 mm | 64 mm |

- The cost increase is real (verified prices replacing guesses), so it's kept, not reverted. **New queue item 5b: cost-down.**

## 2026-11-25: loop 14, cost-down (R8)

Research agent + verified listings (AliExpress prices are search snapshots, so check the live listing before buying):

| Change | Δ cost |
|---|---|
| Bearings from AliExpress 10-packs (6808 $2.02, 6804 $0.92, 6805 $1.15; no-name, inspect on arrival) | −$187 |
| Head camera Orbbec **Gemini 2** ($234; 0.15–10 m, global-shutter IR, 98 g) instead of Gemini 336 ($310) | −$76 |
| Waveshare 2-CH CAN FD HAT ($47) for the arm buses + 1 USB-CAN for base/lift (also the fallback if the HAT's 2nd channel misbehaves on the Pi 5) | −$23 |
| Bestorq HTD belts ($7.67 listing) | −$48 |
| **GIM6010-8 verified price with driver $90–94** (was $82.50 mid-range) | **+$105** |

**Net: BOM $3,645 → $3,435** (target $3,300 ❌ by $135; limit $5,000 ✅).

**Evaluated and rejected:** GIM6010-8 instead of DM-J4310 at the wrist (−$124). The extra 90 g at the arm tip drops the workspace margins of shoulder roll (1.51 → 1.47×) and elbow (1.54 → 1.48×) below 1.5×, and the wrist gets bulkier.

**Notes:**
- The OAK-D Lite rose to $269 (Jul 2026). The RealSense D435i is uncertain supply.
- SteadyWin is phasing the GDS driver out for GDK (pre-order). Check the variant at purchase.

**Remaining gap options (owner choice, non-blocking):**
- accept ~$3.4k
- drop the optical-flow sensor (−$30; rely on wheel odometry + lidar)
- single wrist camera (−$30)
- negotiate GIM volume pricing (Alibaba lists from $39.90; quote needed)

## 2026-11-25: loop 15, owner: "more sensors if it increases accuracy and dexterity" → sensor pack v2 (D9)

- **New analysis** `rnd/accuracy.py` → [rnd/accuracy.md](rnd/accuracy.md): a static hand error budget at 0.6 m reach.
  - The belt joints carry GIM backlash ÷ ratio + belt tooth play + belt stretch under the sustained load. This is 0.19–0.25° per joint, **4.0 mm RSS at the hand**.
  - **Joint-side absolute encoders** (AS5048A on the joint hubs) measure after the gearbox and belt, so the servo removes those errors: **0.9 mm**.
  - Remaining unmeasured: bone bending 1.8 mm and mast sway 1.5 mm (worst static). The wrist camera closes the final loop on objects.
- **Sensor pack v2 (+$276, ESTIMATE prices):**

| Sensor | Count | Why |
|---|---|---|
| Joint-side encoders | 9 | the 8 arm belt joints + lift |
| RP2040 CAN sensor nodes | 3 | one per arm, one in the base |
| Global-shutter wrist cameras | 2 | no blur on moving grasps; instead of $30 rolling-shutter |
| Magnetic-elastomer fingertip tactile pads (AnySkin/ReSkin principle) | 4 | slip and contact sensing for dexterity |
| Palm ToF | 2 | pre-grasp alignment |
| Base cliff ToF | 4 | stair edges |

  The optical-flow sensor and both wrist cameras are kept (the owner declined those savings).
- The electrical plan adds the sensor nodes (~+10 % load per CAN bus).
- **R8:** BOM **$3,435 → $3,711**. The internal target is raised **$3,300 → $4,000** to honour the owner's direction; the $5,000 limit is unchanged.
- New scorecard row: hand error **0.9 mm** (4.0 without encoders) ✅.

## 2026-11-25: loop 16, gripper v2 (dexterity, following the owner's sensor decision)

- **Evaluated:** a parallel rack-and-pinion gripper (UMI-style). Its servo and racks must sit below the base plate, which adds ~40 mm to the hand (l_hand 0.15 → ~0.19 m) and costs wrist torque and reach. **Rejected.**
- **Chosen:** keep the SO-101 jaw kinematics (fixed jaw + servo-driven jaw, 0.15 m) and fit **fin-ray TPU fingers** on both jaws.
  - The compliant contact face wraps around objects and self-centres them, cancelling the non-parallel jaw motion.
  - Precedent: Festo FinRay, the Stanford UMI gripper.
  - The **tactile pad** (MLX90393 behind a magnetic elastomer) sits in the fixed finger's face; the **palm ToF** sits beside the wrist camera.
- CAD: `fin_ray()` generator (thin TPU walls + slanted ribs) in `rnd/parts/hand.py`. Interference clean (bonded joints only). Render: [img/gripper_v2.png](rnd/img/gripper_v2.png).
- Hand mechanism 0.31 → 0.30 kg (TPU fingers are lighter than the rigid jaws). Twin and sim unchanged: the sim already models compliant, coupled fingers.

## 2026-11-25: loop 17, compute and I/O budget (D10)

`rnd/compute.py` → [rnd/compute.md](rnd/compute.md) (stream rates and CPU loads are ESTIMATEs):

| Load | Value |
|---|---|
| Head RGB-D | 183 Mbit/s on USB3 #1 (4 %) |
| Two global-shutter wrist cameras (MJPEG) | 160 Mbit/s on USB3 #2 (3 %) |
| Lidar, CAN A, TTL, audio | ~3 Mbit/s on USB2 |
| Arm CAN B/C | via the SPI HAT |
| **Camera ingest + 500 Hz servo targets + 100 Hz IK + state estimation + ROS 2** | **≈ 73 % of the Pi 5's four cores**: workable but tight |

**Learned policies on the Pi 5 CPU:**
- an ACT-class policy runs at ~1–3 Hz (needs ≥ 30 Hz)
- diffusion policies and VLAs don't fit

**Decision D10:** the Pi 5 stays the real-time/I/O computer. The torso **reserves a bay** (105 × 95 × 40 mm, clash-checked against the pitch modules and carriage) plus a **19 V 3 A branch** and Ethernet for a Jetson-class AI computer (Orin Nano Super ≈ $249, 67 TOPS). It's bought in the AI phase, an owner decision, and isn't in the BOM.

## 2026-11-25: end of the owner-requested run ("keep going until you are done"): build package refreshed

- **Build guide:** sensor installation step added (≈ 2.7 h) → first build **22 h** (one arm ≈ 5 h).
- **Print plan:** 58 pieces including the TPU fin-ray fingers → **12.1 kg PETG/TPU, ≈ 281 h on the P2S** (bigger rounded torso and covers since the looks pass).
- **BOM:** **$3,750** (target $4,000 since D9, limit $5,000).
- **Checks:**
  - 58/58 printable, 0 clashes
  - **twin v1 re-run on the current design:** fetch task ✅, 8/8 robust, peak actuator use 33 %, tip margin 75 mm
  - reach 99 %, workspace margin 1.52×, hand error 0.9 mm
- **Render:** [img/robot_v3.png](rnd/img/robot_v3.png).

## 2026-11-25: loop 18, coupon round C1 package + M1 bench-arm kit

**Picked:** the build-enabling, risk-retiring step from the confidence assessment: test the riskiest new mechanisms for filament only.

- **`rnd/coupons.py` → [COUPONS.md](COUPONS.md), STLs in `rnd/out/coupons/`. Nine coupons, sliced: 267 g, ≈ 7.5 h on the P2S (ESTIMATE):**

| # | Coupon |
|---|---|
| C1 | Print-in-place KX74 socket + collar (pins and spring dropped in at pauses) |
| C2 | KX male face (balls embedded at a pause) |
| C3 | Print-in-place clearance ladder (0.20–0.40 mm) |
| C4 | Press-fit ladder for the dowel pins and balls |
| C5 | Eccentric bushing |
| C6 | HTD 5M 20T/40T pulleys |
| C7 | TPU fin-ray finger |
| C8 | Ratchet segment under 15 kg |
| C9 | GIM bolt-circle coupon (needs a motor) |

  Each coupon comes with a measurement, a pass criterion and the parameter it updates.
- **`rnd/m1_kit.py` → [M1_BENCH_ARM.md](M1_BENCH_ARM.md):** the one-arm bench kit, **≈ $1,096**. This corrects the ~$750 quoted to the owner earlier, which was a guess before itemising.
  - Contents: 4 GIM + DM-J4310 + 2 STS, bearings, KX hardware, belts, CF tubes, V-band, balancer, the arm's sensor pack, wrist camera, adapters, bench PSU + e-stop, filament.
  - Everything except the PSU is reused in the full robot.

## 2026-11-25: confidence campaign (owner: "I need the percentages of confidence higher, run some more simulations")

- **Monte Carlo of the dynamic fetch task** (`rnd/mc.py`, 4 workers): 100 runs with derate, mass, friction, calibration, μ, can mass and position errors.
  - Result: **100/100 (95 % LB 96 %)**, actuator use ≤ 47 %, tip margin ≥ 68 mm.
  - A first pass showed −10 mm tip margins. Cause: `apply_perturb` scaled `body_mass` without refreshing MuJoCo's cached `body_subtreemass`, which skewed the subtree COM. Fixed.
- **FEA** (`rnd/fea.py`, gmsh + scikit-fem) of 7 critical printed parts: KX74 collar, elbow housing, elbow cassette plate, wheel pod, shoulder bracket, roll link, clevis/yaw block.
  - **It found the elbow small-pulley hub unsupported.** Its journals were 30 mm from the walls, and belt peak loaded the 1.2 mm shell (28 MPa, 14 mm).
  - → **Elbow belt cassette:** 2 × 7 mm plates on their own thin 6708 bearings, eccentric bushings, M5 posts, anti-rotation pin in a radial slot. It goes in through a new screw-on **bone cap**. +0.08 kg per arm (params; 6708 thin bearings instead of 61808 keep the shoulder-roll workspace margin at 1.50×).
  - Also fixed: the 40T elbow pulley was 9.4 mm out of the belt plane (`BELT_Y`).
  - The clevis bottom yaw plate was a free solid: 2 posts added.
- **Roll/yaw stage journal constants re-derived from the hub geometry.** The old values predated the taller print-in-place hub: ~20 cm³ of hub/wall clashes, belts 2.6 mm out of plane. ZF +2.4 mm keeps both plate clearances.
- **Thermal** (`rnd/thermal.py`): hottest motor 55 °C on a non-stop task day in the MC worst corner; worst-pose holds unlimited (ESTIMATE constants).
- **New `rnd/integration.py`:**
  - **18/19 cartridge torque pins land in air.** No stage models the slot. → queue item 1, a standard bay-ring/strap design.
  - Clashes inside modules, which the whole-robot check skipped: shoulder stale-hub (queue item 2); elbow collar/forearm (queue item 3).
- Confidence table: [rnd/confidence.md](rnd/confidence.md).
  - Overall first build ≈ 55 % today, ≈ 65–70 % after queue items 1–3.
  - Staged build ≈ 90 %.
  - KX and creep stay capped until the C1 coupons.

## Loop 19 (2026-11-26): standard torque hanger; wheels done

- **Picked:** queue item 1, the cartridge torque-reaction path (the biggest open integration gap).
- **New standard part `cartridge_m.torque_hanger(reach)`:**
  - a 7 mm printed plate with an **open radial fork** for the Ø6 torque pin, so the cartridge keeps its radial float
  - a flange that screws to the nearest flat (2 × M4 into heat-set inserts)
  - designed for the GIM peak: 11 N·m / r 30 = 367 N
- **Wheels ×4:** the hanger screws under the deck; the fork is open downward, keeping 1.2 mm above the battery. FEA: 2.1 MPa → **5.7×**, 0.08 mm. No clashes.
- **Integration check made stricter:** the pin tip must be in clear space with structure on both tangential sides.
  - This exposed two false passes: the head-tilt pin, and the wrist-pitch pins, which run *into* the forearm end wall.
  - **Torque pins 1/21 → 4/21** (honest count; the 1 was false).
- Scorecard 33 ✅ / 3 ❌, unchanged.
  - Same 3 failing rows: R10, torque pins, module clashes.
  - BOM $3,772 → $3,776; 62/62 printable; 0 cross-module clashes; FEA 8 cases ≥ 3.2×.
- **Next:** hangers for the lift and the arm stages (pitch/roll/yaw/elbow), and radial slots for the wrist and tilt pins.

## Loop 20 (2026-11-26): lift torque fork

- **Picked:** queue item 1, the next cartridge on the torque-reaction list (lift).
- **Problem:** the lift cartridge's pin points at the mast and sticks out behind the mount block; the cover left only ~1 mm of room. A screw-on hanger doesn't fit.
- **Fix:** the fork is now part of the printed lift mount (`upper.lift_mount`):
  - a 5.7 mm arm beside the can (can r 43.3)
  - a 7 mm fork plate with an open radial slot
  - load is vertical; hand calc 3.4 MPa at the GIM peak 367 N
- **Cover:** y0 −88 → −91, so the pin tip clears it.
- **Clashes:** no new ones. The belt-render box and the cover-at-mount-face overlaps were already there.
- **Torque pins 4/21 → 5/21.** Scorecard 33 ✅ / 3 ❌, unchanged; 62/62 printable; 0 cross-module clashes.
- **Next:** shoulder pitch (with the queue item 2 wall re-derivation), roll, yaw, elbow.

## Loop 21 (2026-11-26): shoulder pitch walls re-derived + torque fork

- **Picked:** queue item 2, plus the shoulder entry of item 1.
- **Walls from the hub geometry.** They were stale constants, the same issue as roll/yaw:
  - W_IN −43.8 → −41.7; W_OUT −17.9 → −15.55; belt plane −31.2 → −28.6 (the 60T pulley follows)
  - walls 8 → 7 mm, to fit the 7.4 / 7.1 mm gaps
  - removed the 10.4 cm³ hub/wall clash and the outer-wall/pulley-flange overlap
- **Torque fork.** A deep side plate runs inboard from the inner wall to a 7 mm fork plate.
  - It first clipped the Ø98 KX collar (3.4 cm³); the clearance cut was widened to r 50.5.
  - That leaves two 18.5 mm flanges (≈ 4.6 MPa, hand calc). The collar is still turned from the open −X side.
- **FEA:** shoulder bracket 6.5× → 5.9×, 0.33 mm.
- **Checker fix:** `rnd.integration` took the pin position from the hole face's centroid, which is off-axis for a partial cylinder. It now projects onto the true axis.
- **Results:**
  - Torque pins 5/21 → **7/21**.
  - Module clashes 9 → **8**.
  - Scorecard 33 ✅ / 3 ❌. Workspace margin 1.50×, 62/62 printable, 0 cross-module clashes.
- **Next:** roll ×2 and yaw ×2 torque reaction (roll link / clevis), then the elbow (queue item 3).

## Loop 22 (2026-11-26): roll and yaw torque forks

- **Roll** (`roll.roll_torque_fork`): the inner wall rises to z 147. A 6 mm plate (85 mm wide) runs over the can, above the KX collar (r 49.2), to a 7 mm fork plate open downward.
  - It stays > 105 mm from the roll axis, outside the swept envelope of the clevis and yaw block.
  - The cosmetic shoulder cap grew 8 mm back and up so the fork stays inside it.
- **Yaw** (`roll.yaw_torque_fork`): a 7 mm fork plate fused to the inboard clevis arm, 21 mm cantilever.
  - The first version cut the roll-shaft envelope along its whole length and removed its own slot walls. The new check caught it; the cut is now limited to x ±30.
- **FEA:** roll link 6.0× → 6.5× (0.61 mm), clevis 3.2× (1.87 mm).
- **Results:** torque pins 7/21 → **11/21**. Module clashes 8, unchanged; no new overlaps. Scorecard 33 ✅ / 3 ❌.
- **Next:** elbow (with queue item 3: collar and forearm clashes), then wrist pitch/roll and head pan/tilt (these need slots in walls the pins currently run into).

## 2026-11-26: power budget refreshed (owner question: battery life)

- `budget.power_budget` was stale (pre-D9/D10, and it counted 9 GIMs instead of 13).
- **Electronics:** 13.5 → 17.3 W (Pi 5 loaded, Gemini 2, wrist cameras, lidar, sensor nodes, ×1.1 DC-DC losses).
- **Actuator idle:** 13 GIM × 0.8 W + 8 servos × 0.6 W.
- **Average:** 47 → 52.7 W; **runtime 4.0 → 3.6 h** on the 222 Wh pack (85 % usable). R7 ≥ 2 h still passes.
- With the future AI computer (~12 W average, ESTIMATE): ≈ 2.9 h. Non-stop manipulation: 2.4 h (2.1 h with AI). Standby only: 5.8 h.

## 2026-11-26: self-docking charger added to the queue (owner request)

- Queue item 4. Spec covers: rear-skirt contacts with a handshake pin (dead until identified), printed funnel dock, 6S CC/CV charger ~4–5 A, lidar profile + AprilTag homing, approach Monte Carlo, contact FEA.

## 2026-11-26: D11 head face screen adopted (owner)

- Decision D11: a bar-format LCD in the visor, run by its own ESP32-S3 node on CAN so it works if the Pi hangs. It shows the face and status and gives touch controls, including STOP.
- The hard-wired e-stop remains the safety device.
- Queue item 5: CAD, BOM, wiring, power, mass/sway re-check, CAN command set.

## Loop 23 (2026-11-26): elbow housing rebuilt around the real collar; elbow torque arm

- **Picked:** queue item 3 (elbow clashes) + the elbow entry of item 1.
- **Root causes:**
  - The housing's soft corners (r 34) left only ±41 mm inside near the −Y wall; the can is ±43.3.
  - The KX74 collar isn't round: lugs reach r ≈ 52 and the twist tab r 57. It turns 0 → +35° (locked → unlocked).
  - The +Y bearing boss stuck out 4 mm into the forearm disc.
  - The forearm's CF socket and tube swing within 3 mm of the +Y wall.
- **Fixes:**
  - **Housing 98 → 110 mm wide, top +8 mm.** Corners r 14 on the −Y (cartridge) half, r 34 on the +Y (forearm) half, built as two shell halves.
  - A rectangular **twist-tab window** in the +X wall, which is also where you reach in to lock the collar.
  - +Y boss moved inside the wall.
  - **Forearm link +6 mm** (FORE_Y 66 → 72; the elbow hub is 12 mm longer). The hand stays on the centre plane.
- **Elbow torque arm:**
  - The can is clocked 180° so the pin (and the cable window) point up.
  - Two 14 × 6 mm rails fused on the housing top, beside the CF neck socket, carry a 7 mm fork plate open downward (≈ 6.7 MPa hand calc).
  - A first 60 mm solid plate weighed 46 g; the rails are 27 g.
- **Sweeps:**
  - The collar is clear through its whole 0 → 35° twist.
  - The forearm is clear over the full elbow range −150 … 0° (the old housing clashed across the whole range).
- **Mass from CAD:** a full upper-bone roll-up gives 0.647 kg; `bone_mass_upper` 0.66 → 0.65. The yaw KX socket had been double-counted in an earlier estimate.
- **FEA mesher fix:** zero-volume sliver tets are now dropped (they made the solve singular/NaN). Housing 4.4×, 1.91 mm.
- **Results:**
  - Torque pins 11/21 → **13/21**.
  - Module clashes 8 → **4** (remaining: 3 small gripper-jaw overlaps + the 0.3 cm³ yaw-shaft sliver).
  - Scorecard 33 ✅ / 3 ❌: workspace margin 1.50×, twin success 8/8, 62/62 printable, 0 cross-module clashes. BOM $3,788.
- **Next:** wrist pitch / wrist roll / head pan + tilt torque reaction (slots where the pins now run into walls), then queue item 4 (docking).

## Loop 24 (2026-11-26): wrist-pitch torque pads

- **Picked:** queue item 1, wrist pitch (cartridge D, Ø5 pin at r 22). The pin ran into the forearm's 1.2 mm wrist-housing end wall.
- **Fix:** a 7.2 mm pad outside the end wall, fused to the can rim, with a radial slot through pad and wall.
  - DM-J4310 peak 7 N·m / r 22 = 318 N → ≈ 8.8 MPa bearing (a slot in the bare 1.2 mm wall would have been ≈ 53 MPa).
  - The pad adds 6 mm to the wrist width on the +Y side.
  - No new clashes; collision hulls regenerated.
- **Checker fix:** the build123d/OCC distance from a point to a solid is to its nearest *surface*, even when the point is inside. The fork test now scales its probe points to the measured pin radius (Ø6 / Ø5 / Ø4), instead of a fixed ±3.5 mm that only suited Ø6 pins. All earlier passes still pass.
- **Results:**
  - Torque pins 13/21 → **15/21**.
  - Scorecard 33 ✅ / 3 ❌: reach 99 %, twin 8/8, workspace margin 1.50×, 62/62 printable, 0 cross-module clashes.
- **Next:** cartridge S (wrist roll ×2, head pan/tilt). Then queue item 4 (docking charger).

## Loop 25 (2026-11-26): small-servo torque slots; torque reaction complete (21/21)

- **New `rnd/parts/torque.py` → `with_pin_slot(can, anchor, …)`:**
  - finds the pin from the can's seat geometry
  - adds a slotted pad around the tip, plus a Ø10 bridge to the anchor if they don't touch
  - cuts the slot through pad *and* anchor (the pan pin had been boring into the solid head bracket)
  - keeps the can itself clear
- **Applied to:** head pan (head bracket), head tilt (yoke), wrist roll ×2 (palm).
- **Gripper servo sleeves:** no pin by design (screwed into the rectangular pocket); the check now records that.
- **Results:**
  - Torque pins 15/21 → **21/21**.
  - Scorecard **33 → 34 ✅, 3 → 2 ❌** (R10 build time, module clashes 4).
  - No new clashes in the head or hand.
- **Queue item 1 closed.**
- **Next:** queue item 4, the self-docking charger.

## Loop 26 (2026-11-26): self-docking charger, approach sim + dock CAD

- **`rnd/dock_sim.py`:** planar MuJoCo Monte Carlo.
  - Model: a 25 kg R 0.25 disc on velocity servos (wheel force limits).
  - Each run: start ±0.3 m / ±20°, dock-pose estimate error σ 10 mm / 1.5°, wheel slip ×0.9–1.1, lateral drift σ 10 mm/s.
  - Approach: pre-dock 0.35 m out, then drive in at 0.10 m/s with a 60 N push limit. After first contact the lateral axis is current-limited to 5 N, so the V centres the base.
- **What it took:**
  - V faces ±40° → **±60°**, 250 mm long. With ±40° and 200 mm faces, a R 250 base touched the face ends, not the faces.
  - Pads 40 → **60 mm** wide, because heading error dominates the pad's lateral error.
  - Fixed three test-harness bugs: floor contacts counted as dock contacts, 2 contact points per wall double-counted, the seat formula.
- **Result: 100/100 dock (95 % LB 96 %).** Worst pad error 18.8 mm vs ±25 mm capture; worst yaw 4.4°.
- **`rnd/parts/dock.py`:** 2 wings + contact tower (2 leaf contacts at 60 / 100 mm, ID pogo pin at 40 mm, AprilTag on top). All fit the P2S.
- **BOM:** +$59 (charger, dock electronics, contacts). Dock walls cut 20 → 10 mm (it is screwed down) to limit print time.
- **Next:** robot-side pads and wiring on the skirt at 45°, contact FEA, wiring-plan entry.

## Loop 27 (2026-11-26): docking charger, robot side

- **Skirt seams** moved from 45°/135°/… into the wheel windows (0°/90°/…). This frees the 45° azimuth for the contacts and hides the seams. The quarters still fit the P2S (124 × 250 × 250).
- **`base.dock_pad_carrier`:** a 6 mm carrier inside the skirt at 45° with flush pockets.
  - Brass pads: + at 60 mm, − at 100 mm (60 × 12), ID at 40 mm (40 × 10). Ø4 wire holes.
  - Heights match `rnd/parts/dock.py`; azimuth matches `dock_sim.PAD_AZ`.
- **Wiring plan (`rnd/electrical.py`):**
  - Charge path: dock pads → ideal-diode reverse-polarity guard → 7.5 A fuse → BMS charge port; charger 25.2 V CC/CV, 5 A.
  - Handshake: the dock stays off until the base sensor node answers on the ID pad.
  - **The robot commands the charge stop at 4.15 V/cell.** It may run while charging (idle 1.4 A of the 5 A), so a stock charger might never terminate.
  - Open item: confirm a common-port BMS before buying.
- **Contact loads (hand calc):** leaf springs ~5 N on the pads; the 60 N push on the 10 mm dock wings gives ≈ 0.7 MPa. No FEA needed.
- **Results:** scorecard 35 ✅ / 2 ❌, unchanged. 64/64 printable, 0 cross-module clashes, BOM $3,866.
- **Next:** queue item 5, the head screen (D11).

## Loop 28 (2026-11-26): D11 face screen in CAD

- **Panel:** **Waveshare ESP32-S3-Touch-LCD-4.3B** (sourced: waveshare.com SKU 27848, $36.99).
  - 4.3" 800×480 capacitive touch, onboard CAN transceiver.
  - **7–36 V input**, so it runs straight off the 24 V bus.
  - Isolated DI/DO; one DI senses the e-stop loop, so the screen can show the stop state even if the Pi is down.
  - No bar-format ESP32-S3 board was found; the 4.3" sits centred in a dark bezel that spans the face.
- **Head CAD:**
  - Shell 16 mm taller (upward; overall height +16 mm).
  - Flat dark bezel 196 × 86 with a 97 × 56 window; the board sits behind it.
  - Eyes drawn on the screen; eye LEDs dropped.
  - The Gemini 2 moves below the bezel, with its own window.
- **Existing head clashes fixed:** the shell clipped the tilt sleeve (0.4 cm³) and, at +30° tilt, the pan sleeve. The shell now subtracts both servos' swept envelopes over tilt ±30° and any pan angle.
- **Wiring:** always-on LOGIC 24 V, F 1 A, CAN A drop at the head (< 1 % bus load). The screen works when the e-stop has dropped motor power.
- **Results:**
  - head_mass 0.82 → 0.94 kg.
  - R12 tip 64 mm (unchanged); mast sway 1.4 → 1.5 mm (≤ 2).
  - Runtime 3.6 → 3.4 h; BOM $3,866 → **$3,904** (target $4,000: $96 left).
  - Scorecard 35 ✅ / 2 ❌.
- **Next:** queue item 6 (remaining confidence sims: KX Monte Carlo + creep, threshold/carpet Monte Carlo) and the head addition to the module-clash check.

## Loop 29 (2026-11-26): KX coupling Monte Carlo + creep

- **`rnd/kx_mc.py`:** 200k draws per interface.
  - Inputs: spring rate ±7 % σ, compression σ 0.07 mm (printed seats), β 20° ± 1° σ, ramp μ 0.12–0.30.
  - 10-year PETG creep in the preload loop: E(10 y)/E 0.45–0.6 at room temperature, 0.30–0.40 for joints next to a working motor at 55 °C (all ESTIMATE).
- **Results:**
  - **P(slip < rated) = 0 %** for KX74/GIM, KX56/DM-J4310 and KX44/STS. The couplings never slip in normal work.
  - Twist p95 ≤ 2.9 N·m (rule 3).
  - **After 10 years of creep, 15–26 % slip below actuator *peak*** (warm joints lose ~7.5 % preload). That is the designed overload-clutch behaviour; the fetch MC never exceeds 47 % of peak, and the joint-side encoders detect a slip.
- **Actions:**
  - Coupon C1 now ends with a **72 h creep hold at 50 °C on the P2S bed**, then re-measures slip.
  - Queued design option if C1 loses > 5 %: a softer, longer-stroke preload spring.
- New scorecard row (passes).
- **Next:** threshold/carpet Monte Carlo; add the head to the module-clash check.

## Loop 30 (2026-11-26): threshold / carpet Monte Carlo

- **`rnd/threshold_mc.py`:** 80 runs of the roller-level omni model (80 free rollers), 4 workers.
  - Inputs: threshold 5–18 mm, heading 45° ± 10°, speed 0.12–0.18 m/s, hard/carpet, μ ×0.6–1.1, mass 24.6–29.6 kg, COM 0.35–0.50 m.
- **Results:**
  - **Within the 15 mm design limit: 55/55 cross (95 % LB 93 %).** Beyond it, 15–18 mm: 25/25.
  - Torque margin (11 N·m / peak) median 1.58×. 2 runs saturated the GIM peak but still crossed: one at 0.18 m/s (above the D2b 0.15 rule), one on low-friction floor μ ×0.61. Keep D2b.
- **Harness fix:** the time limit was too tight (rollers slip on carpet, so progress runs up to ~30 % slower than commanded).
- **Confidence:** base/carpet ~60 % → 80 %; KX couplings → 60–65 % (loop 29).
- **Results:** new scorecard row (passes). Scorecard 37 ✅ / 2 ❌.
- **Next:** add the head to the module-clash check, then a fresh design critique (queue items 1–6 are done or waiting on the owner/coupons).

## Loop 31 (2026-11-26): module-clash check widened; every module clash-free; fresh queue

- `rnd.integration.module_clashes` now also covers the base, upper (mast/lift/torso/head) and the head at tilt ±30°.
- **Fixed what it found:**
  - **Lift belt ran through the lift mount block** (0.4 cm³) → belt slot. The first slot was misplaced: `MIN` in upper.py centres x/y.
  - Lift cover overlapped the mount → the cover subtracts the mount.
  - **Gripper moving jaw rubbed the base and the servo sleeve through its sweep.**
    - Lever moved outside the base and shortened.
    - Mount plate set 22 mm under the shaft, on its −x side only (a point r below the shaft rises to −r·cos θ as the jaw opens under the servo).
    - Fin-ray top 19.5 → 25 mm under the shaft.
    - Clear 0–45°.
  - Wrist camera +2.5 mm off the fixed fin-ray.
  - **Clevis arms were 1.2 mm inside the rotating Ø40 yaw shaft** → 1 mm clearance cut.
- **Results:**
  - Module clashes 4 → **0** with full coverage. Torque pins 21/21.
  - **Scorecard 38 ✅ / 1 ❌** (only R10 build time, owner decision).
  - Twin 8/8, reach 99 %, 64/64 printable.
- **Fresh critique → new queue (RND_PLAN §4):**
  1. cable routing + harness kit (the biggest R10 lever)
  2. R10 levers
  3. stiffness/hand-accuracy FEA chain
  4. BOM cost pass ($96 headroom)
  5. electronics thermal in the torso
  6. twin v2

## Loop 32 (2026-11-26): cable routing across the joints (harness analysis) → D12

- **`rnd/harness.py`:** per joint, compare torsion through the bore (cable ~±360°/m, ESTIMATE) with a clock-spring spiral, using capacity Δθ = S·(1/r_in − 1/r_out) and bend radius 10 × OD.
- **Finding 1:** no joint can rely on torsion through its bore (free lengths 30–200 mm vs 160–500 mm needed). The electrical note "through the hollow shafts, no slip rings" was not enough on its own. Every joint needs a clock-spring spiral.
- **Finding 2:** the camera's stock Ø3.8 USB cable (bend r 38 mm) fits through neither wrist joint.
  - **D12:** a MIPI-CSI camera on an FFC through the wrist, a CSI→USB bridge on the forearm, and a slim 4-core USB (≤ Ø2.2) onward.
  - Crossable joints: 3/6 → **5/6**.
- **Finding 3 (open):** **the yaw joint has no cavity for a spiral.** The belt runs between the yaw plates, and the plates sit 2 mm from the roll pulley and the arm flange. Queue item 1 evaluates a 12 mm spiral cassette (yaw_drop +12) vs limiting yaw to ±70°, against R1.
- **Harness kit:** segment table (lengths, connectors) in docs/rnd/harness.md.
- **Results:** new scorecard row "cable routing 5/6" (❌) → scorecard 38 ✅ / 2 ❌. An honest new gap.

## Loop 33 (2026-11-26): yaw cable crossing → yaw range ±70°

- **Options compared:**

  | Option | Reach | Bimanual | Shoulder-roll margin |
  |---|---|---|---|
  | baseline (±150°) | 99.1 % | 4/4 | 1.505× |
  | (a) 12 mm spiral cassette (yaw_drop 0.112) | 99.1 % | — | **1.49× (fails ≥ 1.5)** |
  | (b) yaw ±70° | 99.1 % | 4/4 | 1.516× |
  | yaw ±60° | 99.1 % | **1/4** | — |

- **Adopted (b):** twin JR yaw ±150° → **±70°**. The bundle twists between a clamp at the roll-link top and one on the elbow-housing floor: free length ≈ 0.24 m, against 0.194 m needed at ±360°/m (≈ 25 % headroom).
- **Cable routing 5/6 → 6/6.** Queue: printed yaw hard stops at ±75°.

## Loop 34 (2026-11-26): yaw hard stops

- A Ø6 steel stop pin is pressed into the bottom yaw plate (r 38, toward −X). It rides an arc slot in the yaw-shaft flange, with hard stops at ±75°; the software limit is ±70° (loop 33).
- Swept check: clear through ±75°, contact by ±80°. No new module overlaps.
- **Why:** the cable bundle crosses the yaw in torsion (D12). A runaway past the range would overtwist it.
- Fault load: 22 N·m / 38 mm ≈ 580 N → ≈ 19 MPa bearing on the 5 mm flange. Acceptable for a fault-only load.
- Scorecard 39 ✅ / 1 ❌ (unchanged).
- **Next:** queue item 3, the stiffness / hand-accuracy chain.

## 2026-11-26: external design review #1 (Gemini 4 Neon), owner request

- `rnd/external_review.py` sends a 207 kB packet (scorecard, decisions, 20 reports, params.py, all part CAD, 2 renders) to `gemini-4-neon` (OpenAI-compatible endpoint, key injected by the environment proxy, reasoning high, streamed). It ran about 6.5 min.
- **18 findings (7 critical).** Spot-verified 5 in the repo:
  - trapped bearings in small_hub
  - roll stage 20T in CAD vs 18T in belts/BOM
  - wrist 61805s in the mass list only
  - balancer stroke vs cage
  - bumper_light ring blocking the dock pads
- The big new class is **assembly sequence**: our clash checks never asked whether a part can be put in.
- Triage and queue items A1–A16 in docs/review/triage_2026-11-26.md and RND_PLAN §4.

## Loop 35 (2026-11-26): assembly-sequence check → D13 belt stage v2

- **`rnd/assembly_seq.py`** (review #1 finding 1):
  - Bearing slide-on: radius profile along each shaft vs the bearing bore, from either end.
  - Shaft insertion: is a fused element wider than the wall bearing seats?
- **Result: 1/6 drive shafts assemble as drawn** (only the wheel shafts).
  - The small-pulley hub (used ×9) traps its upper 61804 between the Ø87 KX hub and the Ø36 pulley flanges.
  - Pitch, roll, yaw and elbow shafts carry fused pulleys wider than the Ø52 wall seats.
- **D13 (belt stage v2):**
  - Small stages: steel Ø12 shaft + 2 × 6801 + an off-the-shelf aluminium HTD pulley (18T on the roll stage) + a clamp flange coupling bolted to the printed KX74 hub (4 × M4, ≈ 5 MPa).
  - Big stages: the printed pulley is bolted to a shaft flange.
  - A bolted printed-to-printed joint at the Ø20 journal would see > 25 MPa, so it was rejected.
- New scorecard row (assembly sequence 1/6, ❌) → scorecard **39 ✅ / 2 ❌**. The gap was already there; now it's measured.
- **Next:** small-stage v2 CAD (it cascades into J1/J2/walls in shoulder/roll/arm), then the big-pulley split shafts.

## 2026-11-26: owner: half-hour cadence; new targets R14 (assembly simulation) and R15 (spacing)

- **Cadence:** a second hourly scheduled task at :40 joins the :10 one, giving a run every 30 min. A single task can't run more often than hourly.
- **R14 assembly simulation:**
  - bearings slide on from a free end
  - straight-line insertion paths for every part
  - tool access for every screw
  - cartridge swap with the arm assembled
- **R15 spacing:**
  - ≥ 1.0 mm static and ≥ 2.0 mm moving (swept)
  - belts ≥ 3 mm from static parts
  - cable bend radius ≥ 10 × OD
  - no 12 mm finger-probe nip
- Queued at the top, alongside A1.

## Loop 36 (2026-11-26): R15 spacing check

- **`rnd/spacing.py`:** BRepExtrema minimum distance for every near pair, in five classes:
  - mate: no rule
  - running: ≥ 0.3 mm, bearing/shim/guide-defined
  - belt: ≥ 3 mm
  - moving: ≥ 2 mm
  - static: ≥ 1 mm

  R15 is refined to allow bearing-defined running clearances. A flat 2 mm rule would forbid every bearing shoulder.
- **First run: 7 pairs under the hard limit.** Fixed this loop:
  - yaw-shaft top 0.15 mm under the 72T roll pulley (moving) → shaft 2 mm shorter
  - wrist can float 0.3 → 0.8 mm
  - lift-belt slot 1 → 3 mm
  - clevis vs yaw shaft 1.0 → 1.4 mm
  - elbow bay 0.7 → 1.2 mm
  - wrist camera +0.7 mm
  - a mis-classified mate (servo in its sleeve)
- **Results:**
  - Remaining hard violations: **2**, the elbow cassette plates vs their 40T pulley (0.1 mm) and belt (0.03 mm). Both go away with the D13 elbow redesign.
  - Under target: 5.
- New scorecard row (R15, ❌ until D13). R14 row renamed.

## Loop 37 (2026-11-26): D13 small-pulley stage v2 CAD

- **`belt_stage.small_stage_v2(teeth)`:** steel Ø12 shaft, 2 × 6801 straddling the pulley, off-the-shelf aluminium HTD pulley, rigid flange coupling in a Ø32 × 6 pocket under the printed KX74 hub (4 × M4 into inserts).
  - `eccentric_bushing_v2` for the 6801: thinnest wall ≈ 7.5 mm (the review flagged ≈ 2 mm on v1).
- **Why straddled:** a cantilevered pulley would put ≈ 1.4 kN on the near 6801 (C0 ≈ 1.0 kN) at the 691 N belt peak.
- **Stack (`V2`):** bearings −24.7…−29.7 and −57.7…−62.7, belt centre −40.2, end −63.7. That's ≈ 18 mm longer than v1 (−45.6).
- No overlaps between the stage parts. **Passes the R14 assembly-sequence check standalone** (bearings slide on the plain shaft; the pulley is separate).
- **Next:** migrate the stages (elbow first, which also clears the 2 R15 hard violations), then the big-pulley split shafts.

## Loop 38 (2026-11-26): elbow migrated to the D13 small stage v2

- **Elbow uses `small_stage_v2(20)`:** steel shaft, 6801s, aluminium pulley, flange coupling under the KX74 hub.
  - Cassette plates re-derived from `V2`: CAS_A 3.7…9.7, CAS_B 37.7…43.7, bearings flush, 0.5 mm to the pulley.
  - Belt plane y 9.4 → **20.2**; the 40T elbow pulley follows (BELT_Y).
- **R15 spacing:**
  - The spacer posts sat 0.03 mm from the belt → moved to x ±35 with r 7 bosses: 3.0 mm to the belt, 1.1 mm to the +Y housing corner.
  - New mates registered (shaft, bearings, coupling, pulley rotate together).
  - **Hard spacing violations 2 → 0** (7 under target).
- Cassette still slides in from below; module clashes 0; torque pins 21/21. FEA cassette/housing unchanged in margin.
- **Results:** scorecard **40 ✅ / 2 ❌** (R10 build time; R14 assembly sequence, since the remaining stages and the big-pulley shafts are not migrated yet).

## 2026-11-26: software track added (owner); efficiency rules

- docs/SOFTWARE_PLAN.md, starting with the bench arm. Open-data strategy: pretrained open VLA/ACT checkpoints, co-training on Open-X/DROID/LeRobot-hub slices, synthetic twin demos, EEF-delta actions + DROID-style cameras for transfer.
- Phases S0 (sim, now) → S1–S4 (hardware); targets SW1–SW6.
- RND_PLAN §1: runs alternate (:10 hardware, :40 software); efficiency rules (one deliverable per run, incremental checks, background jobs, short logs).

## Loop 39 (2026-11-26, hardware): shoulder pitch → D13 v2

- Pitch cartridge Y_F −58 → **−78** (the 64 mm v2 stack). Walls from V2 (6 mm, bearings flush; W_IN −55.3, W_OUT −21.3); belt plane −41.8; Y_BAL −6 → −3, balancer post 7.8 mm.
- −82 first clipped the lift carriage (3.4 cm³ per can) → −78 leaves 2.3 mm.
- Checks run (incremental): module clashes 0, cross-module 0, spacing hard 0, torque pins 21/21, bracket FEA 6.5× (1.8 MPa, 0.41 mm).
- Scorecard 40 ✅ / 2 ❌ (R10, R14).

## 2026-11-26: owner: R10 has no hard target (still a priority); dexterity check

- R10 → tracked, no pass/fail.
- **Bench IK dexterity map: yaw ±70° (loop 33) cut side grasps 68 % → 40 % and cross-body side grasps 60 % → 2 %.** Reach/bimanual hadn't shown it.
- Queued as the top priority: restore yaw ±150° with a real cable crossing (spiral cassette + 20 g recovery, or a slip ring), plus a dexterity scorecard metric.

## Loop 40 (2026-11-26, owner: "work through the whole backlog before I buy the arm")

One long push instead of half-hour runs. In order:
- **Dexterity:** yaw ±150° restored with a real clock-spring cassette between the split 40T pulley's hub and the bottom yaw plate (9.5 mm cavity); roll cassette on the +X end of the roll shaft. D14 (12 V + TTL made at the elbow node) narrows the ribbon to 7.8 mm. New R16 metric: 34 / 67 / 67 / 57 % (cross-body side grasps were 2 %).
- **D13 on roll and yaw; every big pulley split** (hex / double-D on the Ø40 tubes). R14: all arm shafts assemble. The extra 0.16 kg cut the roll margin to 1.49×; roll 72T → **80T** (ratio 4.44, stock 475 belt, ROLL_Y 75 → 83) restores 1.62×. yaw_drop 0.100 → 0.113.
- **Swept spacing** (new rnd/swept.py): the shoulder cap sat in the path of the yaw block and cartridge at roll 55–95° (true since loop 22). Cap cut by the swept envelope; 0 hard violations.
- **D15 KX socket** (review 2): 6 coil springs in a wider step, screwed retainer, captured balls; KX MC below-peak 26 → 13 %.
- **D16 wrist-roll spindle** (review 5): the KX44 would lift off at 0.6 N·m against 1.3 N·m dynamic.
- **A9 balancer:** the 3 N/mm spring stores ≈ 11 J and could never fit its cage; 2:1 tackle + ISO die spring.
- **A8 P2 FEA + compliance chain** (new rnd/stiffness.py): with quadratic tets the 1.2 mm elbow housing tilted 39 mrad (P1: 5) and the two-plate yaw block 22 mrad: ≈ 37 mm at the hand. Stiffener (4 mm +Y wall and top, gussets) and box walls on the yaw block: 4.3 + 1.5 mm. Thicker housing variants fail to mesh (queued).
- **A6:** can wall up to 63 °C on a task day → ASA cans. **A5:** cat-0 stop, pre-charge, regen clamp (mandatory on a bench supply), watchdog. **A14:** two CAN channels per arm. **A10** tip cases + D18 casters. **A11/D17, A12, A13, A15, A16** in CAD/BOM.
- **Software:** S0.2 URDF/MJCF export, S0.3 LeRobot demos (836 demos/h on 2 workers, expert 100 %), S1 DM-J4310 MIT frames + safety supervisor with fault-injection tests, S0.4a BC dry run. Blocked by the network policy: huggingface.co (pretrained checkpoints) and the GIM6010 manual.
- BOM $4.9k landed with 15 % contingency (over the $4,000 target); M1 kit $1,290 + 15 % = $1,484.

## Loop 41 (2026-11-26, hardware): Astra triage → R14 catches loose pulleys; D19

- Astra review #2b: 22 findings, 5 critical ones verified (pulley bores, KX ball relief, wrist skirt, regen resistor, safety torque path). Verdict: stage the M1 buy.
- R14 now checks every loose pulley/hub bore as well as bearings: **2/6** (the four arm big-pulley shafts fail, correctly).
- D19: integral shaft + pulley dropped into capped U-slot seats (also gives the clevis a real clamp). Queue item 0 lists the Astra criticals.

## Loop 41-sw (2026-11-26, software): safety supervisor v2 (Astra finding 20)

- sw/safety.py v2: bounds the torque the MIT drive will actually produce (kp·e + kd·ė + τ_ff ≤ rated; trims τ_ff, then kp, then kd), caps gains, requires finite feedback ≤ 10 ms old, latches every fault (watchdog, e-stop, NaN/stale feedback, overspeed, tracking, temperature, motor error), reset only with the button released; heartbeat only after a healthy cycle.
- sw/test_s1.py: 3,000 random-policy cycles with random gains → 0 torque-bound and 0 limit/workspace violations; 8 fault types each stop and stay latched.

## Loop 42 (2026-11-26, hardware): D15 male face fixed (Astra finding 5)

- One relief channel per pin track instead of one box over the whole ball: the capture rim at the ball equator is 58 % of the circumference (KX44 53 %), was 0 %.
- balls_mated() now places the balls at z_ball (gap + recess); retainer holes Ø2.5 → Ø4.1 for M3 heat-set inserts. Coupon C2 gets a measured ≥ 20 N pull-out test.

## Loop 42-sw (2026-11-26, software): LeRobot compatibility checked with the official loader (Astra finding 21)

- LeRobot 0.7 rejects our hand-written v2.1 layout. New sw/to_lerobot_v3.py writes the demos through LeRobotDataset.create/add_frame/save_episode/finalize (format v3.0) and reloads them with the official loader: episodes, frame count, 224 px video features and exact action round-trip checked (docs/sw/lerobot_v3.json).

## Loop 43 (2026-11-26, hardware): D16 wrist spindle buildable (Astra finding 4)

- Skirt seat open below + 3 × M2.5 end cap; spindle inserted from above (shoulder on the upper inner race), gripper base bolted to the spindle end (4 × M3) through a spacer boss that clamps the lower inner race: retained both ways. Dog lash 0.4 → 0.1 mm a side.
- Hand module: 0 clashes, spacing clean; R14 gains the spindle row (passes): 3/7 shafts (the 4 big-pulley shafts wait for D19).

## Loop 43-sw (2026-11-26, software): CAN map for the bench arm (Astra finding 19)

- sw/config/can_map.yaml: bus membership, command/feedback IDs, termination ends, node ports (PIO-CAN + MCP2562, SPI encoders, I²C tactile/ToF, half-duplex TTL servos) — the single file drivers and harness read. sw/can_map.py validates it: B1 46 %, B2 46 %, IDs unique, 2 terminations per bus, ratios = params.
- Found the D14 buck under-sized (3 A vs 5.4 A for two stalled STS3215) → 6 A in the M1 kit.

## Loop 44 (2026-11-27, hardware): D19 on the yaw stage

- Yaw shaft one piece again (Ø40 tube + 40T + clock-spring hub); flange is a separate part bolted to the tube end (4 × M4); the bottom yaw plate is its own print, bolted on last (it was fused to the top half since loop 10: Astra #14).
- Tried capped U-slots first: the +X clevis arm stands over the slot and the caps hit the flange and spring ring, so the separable bottom plate is the cleaner fix here.
- R14 4/7 (yaw passes); roll/yaw module clean (only the intended stop-pin press fit).

## Loop 44-x (2026-11-27): payload study for the owner's "20 lb" requirement

- rnd/payload_study.py: planar statics. One hand at reach needs 70 / 42 / 14 N·m (shoulder / elbow / wrist) vs 14 / 9.5 / 3 rated → a different robot.
- Two-handed hug (10 lb per arm, load 0.2–0.3 m out on the forearms): shoulder 12–16 and elbow 12–16 N·m vs 14 and 9.5 rated → holds need ≈ 2× today's continuous torque at shoulder pitch and elbow; brief lifts (< 10 s) work at 0.2 m on peak torque. Corrects the first estimate given to the owner ("same motors, higher ratios").
- M1 motor purchase on hold until the owner picks the scenario.

## 2026-11-27: owner accepts scenario B (20 lb two-handed, held close) → R2c, D20 proposed

- GIM8108-8 (reseller data: 7.5 N·m rated, 22 stall, ≈ 396 g, ≈ $85–110) at shoulder pitch + elbow, both 3:1: hug 1.82× rated at 0.2 m (GIM6010: 0.81×); straight-out 1 kg spec 1.76× (was 1.18×); workspace p95 2.4× / 3.4×.
- Gate: 24 V operation (listings mostly 48 V) and the flange drawing. M1 motor order for those two joints on hold.

## Loop 45 (2026-11-27, hardware): R2c ripple — lift, base, tip with 20 lb aboard

- Lift (GIM6010, 20T): 20 lb moves at 0.3 m/s² need 1.41× rated / 3.1× peak; the pawl holds at 0 W → OK for short moves, no lift motor change.
- Base: 15 mm threshold margin 2.19× → 1.72× (rule 1.5×) ✅. Static tip with the hug load at 45°: ≈ 55 mm, ≈ 32 mm while braking at 0.3 m/s² ✅ (planner rules stay).
- So R2c only changes shoulder pitch + elbow (D20), pending the GIM8108-8 24 V answer.

## 2026-11-27: R2d laundry basket (owner) — rnd/basket.py

- 20 lb basket, hands hooked down into the end handles, full 3D twin statics with the D20 motors: floor pick, carry and bed set-down pass with the basket centre ≤ 0.45 m out (shoulder ≤ 21 N·m: ≥ 3× peak, +10 K over a 2-min carry; tip ≥ 31 mm toward a wheel). Washer/counter top not reachable with this grasp.
- D21: rigid fingertip hooks + wrist-pitch outboard support (the KX56 would see ≈ 2.4 N·m bending vs ≈ 1.5 N·m lift-off).

## Loop 45-sw (2026-11-27, software): load mode in the safety supervisor (R2d)

- sw/safety.py set_payload(kg): above 1 kg the hand workspace shrinks to the box rnd/basket.py passed (load ≤ 0.45 m out, ≤ 0.80 m up), base braking caps at 0.3 m/s², load_heading_ok() checks the load points within ±20° of a wheel.
- sw/test_s1.py: 800 random-policy cycles with 4.54 kg in hand → 0 cycles outside the load box; all earlier S1 tests still pass.

## Loop 46 (2026-11-27, hardware): D21 finger hook (laundry basket)

- finger_hook(): two PETG-CF rails behind the fixed fin-ray (either side of the wrist camera), a 41 × 8 mm foot under the fingertip reaching 9 mm past the jaw face, Ø3 steel pin; ≈ 2.5 N·m at 2× load → ≈ 17 MPa in the print alone.
- Palm ToF moved y 21 → 26 for the rail. Hand module: 0 clashes, spacing clean; moving jaw clears the hook by ≥ 14 mm over its sweep. +19 g per hand.

## 2026-11-27: owner feedback — bigger face, enclosed/pinch-safe body

- D22: 7" Waveshare ESP32-P4 touch display on CAN ($46.99) replaces the 4.3" board; tablet rejected (battery in the head, no CAN, OS upkeep). Head grows; tilt gets a passive pivot.
- D23 + R17: no accessible pinch/nip point (covers ≤ 8 mm openings or ≥ 25 mm gaps over the range), all belts/motors enclosed, lift mast cover + 120 mm bottom gap, TPU bumpers, current-based collision detection. rnd/pinch.py first.

## 2026-11-27: D22 head + D23 concept covers rendered (robot_v8_covers_concept.png)

- Head 252 × 184 mm with the 7" ESP32-P4 board behind a 232 × 120 bezel (156 × 90 window); level and −30° tilt clean, **+30° tilt hits the mast top bracket (1.4 cm³)** → fix with the queued passive tilt pivot.
- rnd/parts/covers.py (render-only, robot.build(covers=True)): shoulder domes, yaw-block sleeves, elbow bay domes + collar-window doors, forearm sleeves, wrist boots, mast bellows. Not yet checked for sweeps/pinch (rnd/pinch.py next).

## 2026-11-27: slim-arm study (rnd/slim_study.py) → D24

- V1 bolted cartridges: motor bumps Ø113 → ≈ 90, elbow 110 → 98 mm, arm 4.26 → 4.03 kg, margins unchanged or better. V2 adds inline GIM8108-36 at pitch + roll: shoulder dome 172 × 210 → 128 × 120 mm, basket margin 1.47 → 2.45, 48 rpm, +$152 per arm, pitch 2.25× stiffer to back-drive.
- D24: bolt every cartridge (owner OK); propose inline GIM8108-36 at roll only (same hand-feel as today's 35.5:1 belt, most of the slimming, +$61 per arm).

### 2026-11-27 — Sprint mode (owner: "it's taking days when it should be hours")
- Cause: one deliverable per scheduled run, then idle until the next firing → ≈ 2 serial items an hour. Now: parallel tracks in worktrees, lead integrates, heartbeats only resume (RND_PLAN §1 "Sprint mode"; both scheduled tasks re-prompted).
- Wave 1 launched (4 engineers): arm CAD (D24 bolted cartridges, D19 pitch + elbow, narrower elbow), shoulder roll (D24 inline GIM8108-36 behind a params switch, D19 belt roll), R17 pinch.py + printable covers + head tilt pivot, non-CAD Astra criticals (A5 v2, balancer sign, clock-spring capacity) + software collision detection + BOM cost pass.
- New `python -m rnd.check`: maps changed files → checks, runs light ones 3-wide, heavy ones serially, logs to rnd/out/check_logs/.
- Fable review pass added (owner): once per day, after a significant merged wave.

### 2026-11-27 — Wave 1, track D merged (non-CAD Astra criticals, software, cost)
- A5 v2: clamp sized for 320 J / 2 kW (2 × RH-50 0.56 Ω + 2 × IRFB7530 at 26.5 V), switched pre-charge, forced bus discharge → torque off in 26 ms, bench catch ≤ 50 mm. M1 kit $1,491 → $1,706.
- Balancer: pin B is opposite the arm in shoulder.py (spring adds to gravity); real ISO 10243 spring is 18.7 N/mm → b 20, a 40–60, 35 % max. shoulder.py change queued for the arm track (exact edit in queue item 0); cable must clear the Ø40 shaft.
- Clock springs with ribbon thickness: yaw 279° vs 360° → r_out 42.5 mm or ribbon ≤ 1.8 mm; two CAN pairs need an 11.3 mm cavity; wrist roll 302° vs 432°. harness 6/6 → 3/6 (now honest).
- sw/collision.py momentum observer: 0 false trips over 9,000 cycles with 20 lb, trips ≤ 18 ms → compliant hold. BOM $5,030 → $4,898 landed.

### 2026-11-27 — Wave 1, track B merged (shoulder roll) + merged-tree checks (rnd.check: 9 checks, 15 min)
- D24 inline GIM8108-36 roll behind `ROLL_DRIVE` (default belt; `PIP_ROLL_DRIVE=inline`): measured dome 117 × 100 mm vs 163 × 219 belt; R14/R15/basket pass; R16 across-body 62 % inline vs 52 % belt (target 54 %; the 2.6 mm cassette drop cost belt 2 points). Awaiting owner OK.
- D19 belt roll: one-piece 80T shaft, capped seats, clevis pinch caps → R14 roll row passes. Cassettes 11.3 mm (two CAN pairs), yaw r_out 42.5 + 0.75 m ribbon → yaw 400° vs 360° ✅.
- Scorecard 39 ✅ / 7 ❌. Open fails are all in flight: R14 pitch + elbow (arm track), R15 head_shell × head_screen 0 mm (pinch/head track), harness wrist roll 302° vs 432° (next wave), R16 belt across-body, R8 cost $4,898, R2b stiffness.

### 2026-11-27 — Wave 1, software bring-up track merged
- sw/bringup.py: M1 day-1 bring-up on python-can (virtual now, socketcan day 1): enumerate → drive torque limit + CAN timeout to flash → e-stop/watchdog tests → hard-stop zeroing → 10 % jog → gravity hold (sets collision thresholds); each step gated; report docs/sw/bringup_<date>.md.
- sw/virtual_arm.py MuJoCo plant with hidden zeros/friction/fault injection; 6 injected faults each caught at the right step (sw/test_s2.py, all pass).
- Fix: DMJ4310 driver treated Damiao status 1 ("enabled") as a motor error — would have stopped the arm on day 1.
- sw/teleop_sim.py: EEF-delta teleop through the Supervisor → LeRobot v3 (5 episodes reload). Day 1 still fills the GIM6010 table in sw/drivers/gim.py (manual blocked here).

### 2026-11-27 — Wave 1, arm track merged; wave 2 launched
- D24 bolted cartridges at pitch + elbow (Ø68 flange, 6 × M4 into inserts, nose spigot, Oldham drive); wrist KX56 → bolted M3 flange. Motor bump Ø113 → Ø86.6; stage 63.7 → 40 mm (mount plane → shaft end).
- D19 one-piece pitch (bolted pitch hub + outer wall) and elbow (bolted seat caps) shafts → R14 6/7 (+ roll from track B). Elbow housing 110 → 98 mm. Forearm double-D pinch clamp; CF bonds ≥ 1 × OD. Balancer pin B moved per track D. P2 hand sag 12.1 → 10.9 mm raw.
- Wave 2 (3 engineers): roll/yaw onto bolted stages + clevis clamps + inline FEA + belt R16 recovery; D20 motor switch + wrist-roll cable + balancer plane + stiffness; M1 kit buy/hold lists + readiness + measured slim study. Covers/pinch track from wave 1 still running.

### 2026-11-27 — Wave 2, buy-list track merged
- M1 kit split: **Buy now $1,401** (incl. 15 % contingency); Hold A GIM8108-8 at pitch + elbow (+$57; seller 24 V answer); Hold B inline GIM8108-36 roll (+$108; owner OK + drawing). Seller questions in M1_BENCH_ARM.md §4.
- D24 in kit + BOM: arm KX socket hardware out; Oldham (D40 12 mm clamp hub + acetal disc, printed motor tongue), inserts, flange screws in. BOM $4,898 → $5,013 landed (+$13 over limit; Oldham $12 ESTIMATE — branded Ruland hubs are $36–47, generic D40 needed).
- Slim study on measured CAD (bump Ø113.8 → 86.6, elbow 110 → 98, stage 64.5 → 40 mm; inline roll +$94/arm at current listings). Print plan 13.8 kg / 325 h (slice cache); one arm 5.63 h hands-on (−25 min).
- Readiness: go on Buy now; GIM CAN protocol is the day-1 blocker (bring-up blocks GIM joints until the manual's table is filled).

### 2026-11-27 — Owner approved the inline GIM8108-36 at shoulder roll (D24)
- After the side-by-side render (docs/rnd/img/roll_inline_vs_belt.png). ROLL_DRIVE default → inline (roll track applying it now, belt kept as fallback via PIP_ROLL_DRIVE=belt).
- M1 kit: Hold B marked approved; waits only on the seller's drawing. Today's plan (GIM6010 at pitch + elbow, inline roll) = $1,869 with contingency; with D20 too = $1,927.

### 2026-11-27 — GIM8108-36 installation drawing (owner-supplied, docs/rnd/specs/gim8108-36_drawing.jpg)
- Frame Ø96 (CAD assumed Ø100); mount 8 × M4 on PCD 84 (assumed 6 × M4); Ø75 rear recess; output Ø30 boss, 3 × M4 on PCD 28.5 + 3 × Ø4; Ø60 step; axial ≈ 29 + 12.5 mm (≈ 42 mm vs the 60 mm ESTIMATE; to confirm).
- Fits the inline roll envelope with room to spare. The small output pattern needs the metal output adapter (3 × M4 at r 14 mm into plastic ≈ 85 N·m < 103 N·m peak). Roll track updating the CAD. Still open with the seller: output-bearing radial rating (≥ 280 N).
- Correction (full installation drawing, docs/rnd/specs/gim8108-36_installation.png): overall ≈ 57 mm with the driver (15.5 boss + 29 body + 12.5 driver section), ≈ 49.5 mm without — matches the listing's Ø96 × 57; the 60 mm envelope stands. Housing 8 × M4 only 3.5 mm deep (screw lengths matter); output 3 × M4 × 4 deep + **3 × Ø4 H7 dowels** on Ø28.5/Ø30 → steel dowels in the metal adapter carry ≈ 300 N·m (peak 103). Output vs driver face to confirm with the seller.

### 2026-11-27 — Wave 1 covers/pinch track merged; wave 3 lift/base/head track launched
- rnd/pinch.py (R17): Ø12 probe + 8/25 mm gap rule over learned joint sweeps, per lift height. Like-for-like 51 joint-nip + 35 exposed → 62 + 31: lift belt, drive and mast gaps enclosed; the rise is one issue (torso/carriage vs lift drive below ≈ 0.46 m, counted once per riding part).
- Printable covers (elbow dome + collar door, forearm halves, wrist boot, torso collars, spine pod, belt duct, mast cap, closed lift cover, base pan + wheel liners): 98/98 pieces fit the P2S. Bellows dropped (no collapsed stack at bottom of travel).
- D22 head tilt −10…+30° on a 6805 friction pivot through the head COM, no clash over pan ±90.
- Open → wave 3 (lift/base/head track): head_screen vs shell R15 hard fail, lift bottom clash (keep lift_min 0.35), lift onto the bolted stage + steel rack/pawl, base interior leak, wheel cans, neck boot. Shoulder TPU boot waits for the inline roll CAD.

### 2026-11-27 — Wave 2 roll track merged: inline GIM8108-36 is the default
- Built to the seller drawing: Ø96 × 57, 8 × M4 × 18 into a cup (2.5 of 3.5 mm thread), Ø30 boss + 3 × M4 + 3 × Ø4 dowels → 7075 adapter → 8 × M5 (PCD 64) into the clevis. ROLL_Y 69, yaw_drop 99.3, dome 121 × 96 mm. Belt kept (PIP_ROLL_DRIVE=belt).
- Shaft → link joints sized: inline boss→adapter 128 N·m (3.65× drive limit, 1.25× stall), adapter→clevis 2.05× limit; belt clevis pinch 1.83×, yaw hex spigot 1.76×, yaw inserts 1.55×. FEA inline roll link 5.6×, clevis/yaw/rear arm 10.8×.
- Belt roll + yaw on D24 bolted cartridges (KX + torque forks gone). Inline baseline: R14 7/7, swept 0/0, R16 60 % ✅, basket 1.50, roll 51.6×.
- Open: insert pull-out + μ coupons; seller: output face, bearing radial rating, 7075 adapter sourcing. Belt-only items deprioritised (belt R16, roll 450 belt, 80T/18T stock).

### 2026-11-27 — Merged-tree checks after the inline default (9 checks, 9.5 min) + check fixes
- Fixed check bugs: belts.py still rated a roll belt for the inline drive (peak pull 7.2 kN "0.5× ultimate" — there is no belt); integration.py now treats the D21 hook-in-jaw and D22 tilt stubs-in-shell as designed interfaces (module clashes 7 → 0).
- Real open fails: R1 reach 94 % (floor pickup 100 → 94 % with inline geometry), R13 lift_cover × carriage at lift 0.40 (both → lift track), R15 head_screen × shell (lift/head track), harness wrist roll (arm track), R2b 11.0 mm raw, **R8 BOM $5,199** (inline roll on both arms +$186 → $199 over the $5,000 limit; needs a cost pass).

### 2026-11-27 — Wave 2 arm track merged; Fable review #1 started
- D20 behind PIP_ARM_MOTORS=d20 (GIM8108-8 ESTIMATE Ø96, Ø50 Oldham, elbow 20T:60T at C 84 in a 109 mm housing; default gim6010). Both sets pass R14/R15/integration/assembly/swept; basket passes on D20 CAD masses (arm 4.36 kg).
- Balancer on an aluminium crank inboard of the pitch shaft (cable CAD-checked clear, K 4.49 N·m, 35 % deflection); pitch harness on an outboard clock spring (352° vs 312°). Wrist roll ±120° → harness 6/6.
- R2b 10.9 → 8.5 mm raw (1.7 mm after the gravity model, ≤ 2 ✅): housing 3.37 → 1.42, bracket 1.62 → 0.99. Remaining: roll link 2.27, yaw block 1.53, CF tubes 1.8. Forearm socket clears by 3 mm over the elbow range.
- Open: D20 elbow belt peak 2.8× ultimate/3 (steel-cord belt or ≈ 20 N·m torque cap). Fable review (once/day, stamp docs/review/.fable_last) running on the merged tree.

### 2026-11-27 — Cost pass merged: BOM $5,308 landed (+$308 over the $5,000 limit) — owner decision needed
- The inline roll had been carried as a bare motor: GIM8108-36 with the 24 V SDC303 driver is $240 (SteadyWin store, aifitlab), not $171; belt-roll parts out, 7075 adapter (ESTIMATE $30) + 61806 + dowels in; sensor nodes 3 → 5 (can_map). Savings: DM-J4310 $125 (AIFit), covers inside the PETG bundles.
- No other sourced saving keeps the approved motors and safety items. Levers: raise the limit, revert to the belt roll (−$368), or defer non-essential sensors. M1 today's plan $1,945.

### 2026-11-27 — Wave 3 lift track merged; Fable review #1 (buy with changes); fix C2
- Lift v2 on a bolted stage in a two-wall bracket + steel rack/pawl (A15); torso bottom at lift − 75 (120 mm clear at 0.35 m); R13 lift clash gone; head screen bay (R15 0 hard); base sealed; neck boot. Pinch 62/31 → 21/20 (left: tilt +30° slot, inline shoulder boot). R1 stays 94 % (lift_min can't drop in this layout).
- **Fable review** (docs/review/fable_review_2026-11-27.md): buy with changes — reduced step-1 kit (one GIM6010, DM-J4310, servos, CAN, e-stop), no extra GIMs until the GIM protocol is proven on one unit. Critical: C1 arm hangs from M3 inserts in pull-out (pitch hub, yaw flange); C2 compliant hold dropped the heartbeat; C3 bolted cartridge can't react torque as drawn + flange screws not reachable (R14 c/d missing). Major M1–M8 (inline boss preload, bench supply 10 A vs 11.4/33 A, no encoder mounts/firmware, GIM protocol, belt centres, hold A incomplete, optimistic thermal, balancer pulley bearings).
- C2 fixed: heartbeat stays up during the collision compliant hold (sw/safety.py + regression assert in test_s1). can_map roll ratio → 1.0 (inline). sw.test_s2 bring-up safety step fails after the inline switch (reset refused after the e-stop test) — pre-existing, → wave 4 software track.

### 2026-11-27 — Wave 4 (Fable review fixes) merged after two container restarts
- **C1** tie-rod joints: pitch hub ↔ tube on 4 × M4 10.9 studs into a 7075 crank; yaw flange 6061 + steel clamp ring + 6 × M3 12.9 from the tube top; printed parts only in compression (rnd/joints.py: basket/hug/140 N bump/stall cases; FEA cases added, not yet run). **M1** inline boss torque on the dowels only: 1.60× stall; rear M4 engagement 3.0 mm; steel roll stop pins. **M8** balancer pulleys U604 (C0 0.28 kN) → 608-2Z + POM tyres (s0 1.83).
- **C3** (partial, cartridge track cut off by the restart): D24 v2 cartridge (ears outside the sleeve, M4 through-bolts into nuts, rear cap v2 with inserts in compression), R14 (c) tool-access + (d) swap checks added, elbow AS5048A, exact belt centres (Fable M5), D50/D63 Oldham (M6). Finishing pass relaunched (pitch/yaw/lift/wrist sites, roll/pitch/yaw encoders).
- **M2/M4/M6/M7 + test_s2**: bench supply Mean Well SE-1000-24 (41.7 A) vs 14.8 A sustained / 31.5 A peak; one-shot discharge vs pre-charge; safety-board netlist; watchdog 8 → 25 ms from measured loop jitter (7.9 ms worst); GIM protocol notes + sw/gim_discover.py as bring-up step 1; honest thermal (closed sleeve → 0.65× rated continuous, task-day winding 99 °C; finned Al cap fixes) and torque (2.21 → 1.96×; worst pose 1.20×); inline backlash → hand error 4.3 mm without encoders. sw.test_s1 + s2 pass.
- **M1 kit in two steps** (Fable verdict): Step 1 $1,368 (one GIM6010 + DM-J4310 + servos + CAN + safety chain: prove protocol + safety), Step 2 $1,099; GO on Step 1, NO-GO on Step 2 until Step 1 passes + seller answers + C1/C3 land. Robot BOM $5,480 landed (owner decision pending on the $5,000 limit).

### 2026-11-27 — Fable C3 closed (W4b merged); wave 5 launched
- Every bolted cartridge (pitch, elbow, yaw, lift) on its v2 ear pattern with nuts in compression; wrist can flange through-bolted. **R14 (c) tool access and (d) cartridge swap pass at every joint for both motor sets.** AS5048A mounts at pitch/yaw/inline roll (air gaps 1.0–1.3 mm vs 0.5–2.5 datasheet). D20: AI bay +39 mm, elbow housing cuts, carriage slot (needs FEA). assembly_seq crash on the C1 yaw data fixed.
- All 3 Fable criticals now addressed in CAD/code (C1 FEA still to run). R16 dexterity regressed today (45° 69 → 50 %, across-body 60 → 50 %) — cause unknown.
- Wave 5: (1) dexterity/R1 regression root cause, (2) pinch leftovers (inline shoulder TPU boot, tilt slot), (3) run C1 + D20 carriage FEA, palm through-bolts, swept 1.7 mm lug, balancer sign + spring cage.

### 2026-11-27 — Owner: budget limit $5,500; wave 5 merged; hull + roll stop fixes
- **Owner raised the R8 limit $5,000 → $5,500** (keeps the inline roll and the review fixes; $4,000 stays the aim). BOM $5,480 landed passes.
- R16/R1 regressions traced: R16 = wrist roll ±120 (the metric now treats the parallel jaws as 180°-symmetric with the fixed-jaw grasp centre: 78/78/71 %); R1 = the torso pitch-module hull ended at the detach plane (y 200) while the shell side is at y 195 — fixed in collision.py (TORSO_HALF_W) on top of the twin's roll ≥ −12.5° re-solve: R1 94 → 98 %.
- Roll hard stop −19° → −16.5°: verified, at −19° the yaw band runs 2.52 mm into the arm-side ferrule (ferrule_sweep_gap), +0.38 mm at −16.5°.
- C1 FEA first run: pitch tube + hub 4.7× service / 3.1× fault, yaw tube 5.7× / 6.3× (vs 12 MPa); D20 carriage 8 mm walls 20.2×. Palm → wrist flange through-bolted into captive nuts (no inserts left on the hand path). Swept yaw can ↔ roll stub 1.7 → 2.2 mm. Balancer sweep sign fixed (bracket gap 2.1 → 3.3 mm), spring-cage window closed.

### 2026-11-27 — Owner shared Menlo's Asimov zero-shot sim2real post → software track W6
- Lessons adopted: make the hardware deterministic instead of training the policy to hedge; clean, low-latency CAN (they moved to SPI-CAN MCP2518FD); PREEMPT_RT + cooling + no thermal throttling; "only randomize what's actually random" — measure actuators (real2sim) rather than randomize around them.
- W6 engineer: CAN latency/jitter/drop/corruption injection in the virtual bench (safety + collision thresholds re-checked), actuator sysid step in bring-up feeding the twin, determinism checks (jitter, throttle flags, RT kernel, CAN error counters) + a 2 h soak test as a Step-1 exit criterion, USB-CAN vs SPI-CAN (MCP2518FD) decision note.
- (Owner follow-up) Saved the simulator note from the Menlo post: mjlab for fast iteration, Isaac Lab (Newton) for later-stage physics, sim2sim across both → docs/sw/simulator_options.md; to revisit at S3/S4 (Isaac Lab needs a GPU → owner decision).

### 2026-11-27 — Wave 5 covers merged (R17 25/22 → 21/16)
- Yaw hood now covers the yaw can front (can, motor, cap closed to the probe on both arms); neck boot cut 1.5 mm clear of the yoke over −10…+30° (integration clash gone).
- pinch.py compliant-cover rule: TPU 95A boot closes a gap only with wall ≥ 1.2 mm and every free span within ±40 % of as-printed over the joint range (ESTIMATE until a coupon). It shows no boot can close the inboard roll wedge (≈ 2 → 135 mm over −15…+100°) or a hub-bonded neck floor (+590 %).
- Remaining R17 needs product calls (asked the owner): shoulder wedge → ≈ +23 mm roll offset or a roll soft limit near people; head → neck redesign (raise the head ≈ 70 mm on an outside fork, or lower pivot / less tilt); elbow/wrist → flat forearm inner face (arm.py).

### 2026-11-27 — W6 merged: sim2real determinism (Menlo lessons) + R2b closed
- Virtual CAN realism (sw/bus_sim.py: 0–9 ms each way, drops, duplicates, CRC errors, bus-off, corruption). Found + fixed 4 host bugs (stale buffered frame taken as fresh, CLEAR_ERROR read too early, register reads pausing the heartbeat, no plausibility filter); stale-feedback limit 10 → 20 ms; every profile passes all 7 bring-up steps with 0 false stops; real faults still caught (contact 14–18 ms, silent bus 22–30 ms, bus-off 22 ms). Open: corrupted torque values → ≈ 1 compliant-hold false trip / 20 s on the 'corrupt' profile (interface fix).
- Bring-up step 7 sysid (friction, Kt, backlash, latency → actuators_<serial>.yaml, loaded by twin / bench / virtual arm): rehearsal recovers hidden values within ≈ 5 %. Determinism section in every bring-up report; 2 h soak test = Step-1 exit criterion 7. Step 1 adds a Waveshare 2-CH CAN FD HAT (MCP2518FD, $46.99) to measure against the gs_usb adapters → **Step 1 $1,428**.
- **R2b closed: 8.5 → 5.1 mm raw, 1.0 mm after the gravity model** — roll link side block (2.09 → 0.80), yaw block +X end wall (1.47 → 0.40); model corrections: CF-only tube term (printed boxes were counted twice, −0.88) and the real static wrench. On the old model the new geometry gives 6.1 mm; the elbow housing (1.59) is the next term. +79 g per arm.

### 2026-11-27 — Scorecard 46/0 on the merged tree; owner decisions D25 (R17)
- integration re-run: module clashes 2 → 0 (the yoke × neck_boot rows were a stale report from a merge). **Scorecard 46 ✅ / 0 ❌.**
- D25 (owner): shoulder wedge → software soft limit near people (roll ≥ ≈ +15°); head tilt −10…≈ +15°. Wave 7 implements both and re-runs pinch.py.

### 2026-11-27 — Gemini 4 Neon review #2 (docs/review/neon_2026-11-27/gemini4neon_review.md)
- Two HTTP 502s on 622 / 477 kB packets; the third (docs + params, 239 kB) ran. 5 critical, 10 major, 1 minor.
- Critical: (1) safety-board netlist bugs — inductive RH-50 clamp resistors with no freewheel path, K1/K2 chatter on bus sag (PRE_OK in the hold term), a 40 A fuse that never clears on a 41.7 A supply; (2) C3 left only 2–3 clustered flange ears (yaw/lift 2 screws over a 70–80° arc) + D40 Oldham / printed tongue still in BOM; (3) C1 tie-rod preload lost to PETG creep (30–57 µm bolt stretch) → Belleville stacks or crush sleeves; (4) tip-over: 10/16 A10 cases fail (to −102 mm), D18 casters vs thresholds/doors; (5) R17 not a scorecard gate, D25 soft limit relies on non-safety perception, Cat-0 drop.
- Major: inline roll dowel allowable 150 → 240 MPa unjustified, rear mount creep, hyperstatic 61806; elbow shaft cross-layer bending, yaw belt centre 93 % of adjuster, default config fails R2c/R2d; thermal caps vs rear PCB; FEA method; harness (20 AWG, 3.25 m USB, lift umbilical); BMS 40 A vs 74 A peak; docking; print/assembly time; BOM realism; Step 1 kit content.
- → wave 8 (4 tracks). D25 challenge goes to the owner.

### 2026-11-27 — W8a merged: Neon #2 electrical findings
- Safety board netlist v3: 3 × NH-50 + VS-40CPQ060 freewheel Schottkys (turn-off spike 839 V → 32.7 V), TLV3202 (55 ns) at 26.4 V / 0.6 V hysteresis, K1 holds on AUX and any K1 drop clears LATCH (Neon's chatter mechanism was wrong — PRE_OK is one node once K1 closes — but the board re-armed itself after a drop), U6 on Ā and its clear input fixed (the discharge pulse could never fire), 60 A MIDI main fuse (the SE-1000 latches off at 105–125 %, it doesn't current-limit).
- Battery: pack 35 A cont / 90 A burst; BMS → JK-B2A8S20P (200 A); software cap 70 A peak / 28 A over 30 s (to implement in sw/). Harness: arm pair 20 → 16 AWG ETFE (drop 6.4 → 2.6 V, spiral heating 48 → 19 K), USB pins on the detach plug (10-pin pogo), lift umbilical drag chain, XT60s. Dock: 28 mm pads, wheel cradles, weighted baseplate (CAD to adopt).
- M1 Step 1 $1,428 → $1,387 (obsolete D40/CNC cap/servos out); robot BOM $5,485 → **$5,845 (over the $5,500 limit)**: keyed D50 stages + Oldhams +$111, clamp +$51, BMS +$50.

### 2026-11-27 — W7 merged: D25 implemented
- People-near mode in sw/safety.py: shoulder roll ≥ +5° (pinch: inboard wedge ≥ 25 mm from ≈ +1.2°, 30 mm at +5°; the +15° estimate was conservative), targets clamped, in-zone exit slewed at 0.2 rad/s with no stop, trigger stub on at 1.5 m / off after 2 s beyond 1.8 m, missing perception = person near. 3000 random cycles: 0 below the limit. Cost while on: R16 across-body 71 → 49 %, top-down 35 → 18 %.
- Head tilt −10…+15°, but the tilt nips don't depend on the upper limit: the neck-opening rim sits ≈ 3 mm from the yoke → needs a neck CAD pass (≥ 25 mm round posts/plate/hub, boot over it).
- Pinch 21/16 unchanged by default; 18/16 with people near (roll 7 → 4). Remaining roll nips the soft limit can't close: yaw hood × roll link/ferrule, roll outflange × roll link.

### 2026-11-27 — W8d merged: tip-over (Neon #2 F4) + honest scorecard gates
- A10 v2 (caster take-up modelled, all headings, limits read from sw/safety.py): before −121 mm, 6/16 ≥ 30 mm (the loop-40 −85 mm was optimistic). After: power board into the base belly, **2 × 2.44 kg removable rear steel ballast** (robot 24.8 → 31 kg), D18 → D18b lockable 2" ball casters with 22 mm float (threshold-friendly): 16/16 ≥ 30 mm (worst 33) once supervisor rules SW-T1 (person within reach: base ≤ 0.1 m/s, hands close/low), SW-T2 (casters locked by default) and SW-T4 (thresholds at 0.12 m/s) land; 8/16 with today's software.
- Scorecard now names the default config and has hard gates: R17 pinch (21/16 ❌), A10 (❌ until SW-T1/T2), R2c/R2d with the default GIM6010 ❌ ("pass with D20 only"), R4 threshold 1.45× at 0.15 m/s ❌ until SW-T4.
- Flags: **R5: folded arms span 0.70–0.78 m > a 70 cm door**; BOM +≈ $80 (ballast, casters, solenoids); threshold.py / sim / mc / threshold_mc to re-run at 31 kg.

### 2026-11-27 — W9b merged: battery current cap in the supervisor (sw/power.py)
- Power model (not phase current): I_bat = Σ(k_cu·R·I² + τω/η)/(V_bus·η) + idle + 9.9 A rest; regen offsets only its own joint's copper loss; BMS reading overrides when higher. Cap 70 A peak (scale feed-forward, then gains) and 30 s mean ≤ 28 A (shared torque-limit fraction), never below 1.25 × gravity torque (a derate first caused a tracking-error stop), never stops the arm.
- Tests: random policy asking 86 A held ≤ 70 A; sustained high power capped (30 s mean ≤ 27.94 A); two-arm basket carry 120 s ≈ 26.2 A robot → no derate, but only 1.8 A of headroom (R per phase is ESTIMATE; measure at sysid).

### 2026-11-27 — W9c (door stow) + W8c (Neon F3/F6) merged
- **R5 door stow** (rnd/stow.py, scorecard row): crossing at 45° with the right arm back / left forward → 567 mm wide vs 570 mm limit (650 mm clear ESTIMATE − 40 mm a side); sideways (no threshold) 500 mm; straight-on impossible (roll motors 644 mm). Tip ≥ 82 mm, gaps ≥ 28 mm, valid with people near. Flags: needs Supervisor.set_door_stow (hands leave the workspace box); D2b threshold margin holds only at exactly 45° (1.08–1.27× at 1–5° off); roll-output × torso 13 mm (cover).
- **C1 creep (F3)**: creep model in rnd/joints.py (φ 2 design, ESTIMATE from printed-PETG creep data): printed clamps kept ≈ 30 % preload, yaw gapped (0.86×). Disc springs rejected (they soften the bolts that carry the bending, 0.84× / 0.99×). **Steel crush sleeves (42CrMo4) on every tie and clevis bolt**: life preload 88 / 87 %, no-gap 1.85× / 1.61×. Crank thread 7.6 kN with chamfers (not 11.6), 1.77× with sleeves.
- **Inline roll (F6)**: dowel allowable back to 150 MPa with load sharing → bonded dowels + 3 × M4 at 2 kN = 1.03× stall (was claimed 1.60×); all-metal rear clamp (3 mm 7075 plate, sleeves; engagement 2.5–3.5 mm); 61806 floats ±0.3 mm (forced load ≤ 800 N); stop pins had zero wall → steel stop bushes in the housing (1.81×). **Motor output bearing must take ≈ 612 N in service (not 280)** → seller question. Open: 140 N bump can put 113 N·m on the roll (safe only if the actuator backdrives).
- BOM to update: sleeves, bushes, plate, screws (per arm), no Bellevilles.

### 2026-11-27 — W9d merged: tip-over rules in the supervisor → A10 gate passes
- One people-near mode (set_people_near / set_person_near): D25 roll ≥ +5° + SW-T1 (hands in LOAD_WS ∩ x ≤ 0.35, z ≤ 0.60; base ≤ 0.1 m/s and 0.1 m/s²; base held with > 2.5 kg in hand); fixed a roll catch-up at 1.5 rad/s (now ≤ 0.2). SW-T2 caster lock (locked by default, threshold/transit release refused with a person near or load, 100 ms re-seat pulse, 0.3 m/s locked / 0.5 m/s² released). SW-T4 threshold_speed 0.12 m/s; rnd/threshold.py CROSS_SPEED 0.15 → 0.12.
- **A10 hard gate: 16/16 ≥ 30 mm (worst 33 mm) with today's software ✅.** Scorecard: 50 ✅; hard gates 1/2 (R17 pinch 21/16 ❌). Other ❌: R2c/R2d with the default GIM6010 (pass with D20 only — waits on the seller's 24 V answer), R8 BOM $5,845 vs $5,500.

### 2026-11-27 — W8b merged (Neon #2 F2/F7) + its shoulder/roll edits applied
- 4 flange ears at every cartridge site (was yaw 2/70°, lift 2/80°, pitch 3/110°): yaw side-fed nuts, pitch4 rear pair screwed from the pulley side through the outer wall (wall_b_cuts), lift4 wall towers (upper.py edit pending the neck track). Tipping ≥ 10× at 30 % preload (was 0.8×). D20 ears slip into bearing at stall (≈ 4 MPa).
- Printed ASA Oldham motor tongue rejected (0.74× friction at peak) → 6061 motor-side hub; D50 sets on every GIM6010 stage, D63 for D20. Elbow D19 shaft: bonded 6061 Ø30 × 1.5 core takes ≈ 85 % (print 5.0/14 → 0.7/2.0 MPa). Yaw centre 80.93 mm for a stock 315-5M belt (was 93 % of the adjuster).
- BOM/kit/print plan/build guide consolidation running (W10); then one budget picture for the owner.
