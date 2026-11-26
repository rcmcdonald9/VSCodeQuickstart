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
