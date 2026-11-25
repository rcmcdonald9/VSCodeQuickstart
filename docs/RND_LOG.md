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
