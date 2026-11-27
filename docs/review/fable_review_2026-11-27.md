# Fable design review: Pip v4, before the M1 bench-arm purchase (2026-11-27)

Scope: tree at `280715d`. I read the plan, scorecard, M1 docs and both earlier reviews, then the source (`rnd/params.py`, `rnd/parts/*.py`, the analysis modules, `sw/`). Light modules were re-run (`belts`, `accuracy`), plus three small probes of my own (driver access to the flange screws, the supervisor heartbeat, belt centre distances). No FEA or `run_all`.

**V** = verified in code or by a run. **S** = suspicion from engineering judgement, to be checked on a datasheet or a coupon.

## Verdict

1. **Buy with changes.** Buy a reduced step-1 kit now and fix six design items before printing the whole arm.
2. Step 1 is one GIM6010-8, the DM-J4310, the two STS3215, the CAN adapters, the e-stop parts, the cheap drive parts and filament. Do not take the "zero-regret" three extra GIM6010s yet: no GIM can be commanded until its CAN protocol is known.
3. The arm as drawn hangs from M3 heat-set inserts in pull-out at two joints, and the bolted cartridge's rear cap has no workable thread. Both are CAD fixes, but they come before printing.
4. The approved inline GIM8108-36 roll is the riskiest new element: a 103 N·m stall actuator on 3 mm of M4 thread, printed Ø8 stops, and backlash that the accuracy model leaves out.
5. The bench electronics are a parts list without a schematic, the supply is 10 A against the repo's own 33 A peak, and one supervisor bug would drop the arm after a detected collision.

## Critical

### C1. The whole arm hangs from M3 heat-set inserts in pull-out, twice in series (V)

- **Pitch hub to pitch tube:** `rnd/parts/shoulder.py:62, 113-131`. The hub carries the detach ferrule, so it carries the whole arm. It sits on an 8.3 mm double-D and is pulled on by 6 × M3 into inserts in the printed tube's end face (r 15.5 mm).
- **Yaw flange to yaw tube:** `rnd/parts/roll.py:549-592`. 6 × M3 countersunk into inserts in the tube end (r 16.5 mm, about 1.5 mm of wall to the Ø40 surface). Everything below the yaw hangs on them.
- **Loads:** the repo's own figure for the yaw joint is 644 N per insert with the basket, against an estimated 1.0 kN pull-out, so 1.55× on a guessed ultimate (`drive_capacity()`). The pitch hub has no capacity calculation at all. For the 19 N·m FEA case I get 200–410 N per insert static (pivot at the edge vs the centre) and 300–610 N with the ×1.5 factor. A 140 N sideways bump at the hand is 1.1–2.2 kN.
- **Not covered:** neither joint is an FEA case, and no coupon tests insert pull-out (`docs/COUPONS.md`).
- **Tightening alone can do it:** the inserts go in from the face the screw enters, so a normally tightened M3 (1 kN or more of preload) loads them in the weak direction.
- **Consequence:** the arm, or the forearm and three motors, drops off. This was the fix for Astra finding 2; it moved the problem.
- **Fix:** put the printed tubes in compression. Use through-bolts with nuts, or a hollow central tie-rod through the bore (cables still pass), or steel M4 cross-dowel nuts. Add a pull-out coupon and a capacity row for the pitch hub. Do not print the pitch or yaw shafts as drawn.

### C2. A collision "compliant hold" stops the hardware heartbeat, so the arm drops (V)

- `sw/safety.py:204` sets `healthy_cycle = True` with the comment "keep the hardware heartbeat".
- `heartbeat_ok()` (`sw/safety.py:144-145`) returns `healthy_cycle and not self.latched`, and `latched` is `["collision"]`.
- `sw/bringup.py:346` only toggles the watchdog when `heartbeat_ok()` is true.
- I ran it with a stub detector: `stop False, compliant True, heartbeat_ok False`.
- **Consequence:** 8 ms after a detected contact the watchdog opens the contactor. That is a category-0 drop onto whatever the arm just touched, the opposite of the intent.
- **Why the tests pass:** `sw/test_s1.py` has no watchdog in the rig, and `bringup.supervisor()` (`sw/bringup.py:323`) never passes `collision=`, so the bug is latent.
- **Fix:** `return self.healthy_cycle and set(self.latched) <= {"collision"}`, plus a test with `SimSafetyIO` in the loop.

### C3. The bolted cartridge (D24) cannot react motor torque as drawn, and two joints cannot be swapped (V)

- **Rear cap:** the motor bolts only to the rear cap (3 × M3). The cap is a plain 4 mm disc (`rnd/parts/cartridge_m.py:193-203`) "held by 3 radial M3 into inserts in its rim" (`:187-189`). No holes are cut in the cap, and a 4 mm rim cannot hold an M3 insert (Ø4 hole). The motor reaction therefore has no path to the sleeve.
- **Mounting face:** the design assumes the GIM6010's housing holes are on the rear face (PCD 57, marked ESTIMATE). If they are on the output face, as on many planetary actuators, the cap concept fails altogether. (S)
- **Flange inserts:** they can only be installed from the mating face, because the sleeve overhangs the back. The countersunk M4s then pull the inserts out toward the wall, which jacks the flange loose while the screw feels tight.
- **"A swap is 6 screws with the belt on"** (`cartridge_m.py:124`) is false at the elbow and at the shoulder pitch. I probed a Ø6 driver path along each screw axis:
  - Elbow: all six are blocked by the stiffened +Y housing wall. The ±75° pair also sits under the cassette's M5 spacers, 7.4 mm away (`rnd/parts/arm.py:47`). The D20 variant moved its angles for exactly this reason; the default did not.
  - Shoulder pitch: all six are blocked by the outer bracket wall.
  - Roll (belt) and yaw have driver holes (`flange_screw_access`).
- **R14 overstates:** parts (c) tool access and (d) cartridge swap are not implemented anywhere. `rnd/assembly_seq.py` checks bearing slide-on and shaft insertion only, so the scorecard's "7/7" covers less than the requirement says.
- **Fix:** give the cap an 8–10 mm skirt, or bolt the motor's front face to the step. Use captive nuts behind the flange instead of inserts (there is a 16 mm gap). Add driver holes at the elbow and pitch, and rotate the elbow bolt pattern off the posts. Implement R14 (c) and (d) as a probe like mine.

## Major

### M1. Inline GIM8108-36 roll: joints sized on impossible preloads, weak hard stops, missing backlash (V, part S)

- **Output boss** (`rnd/parts/roll.py:753`): friction is computed with 4 kN preload on M4 screws that have 3.0 mm of thread in aluminium. Line 40 of the same file rates 2.5 mm of that thread at about 1.6 kN. With a realistic 1 kN or less, the capacity is about 110 N·m against the 103 N·m stall (1.07×, not the reported 1.25×), and all of it is dowel bearing in a boss assumed to be aluminium.
- **Rear mount:** M4 × 18 through 15.5 mm of plastic into holes tapped 3.5 mm deep. The engagement is 2.5 ± 0.55 mm, so the screw either has under three threads or bottoms out. The loads are low (about 100 N per screw), but it is easy to strip a $215 motor. Use Loctited studs and nuts.
- **Hard stops:** two printed Ø8 lugs (`roll.py:305`) see 800 N at the 35 N·m software limit, about 16 MPa against the 12 MPa allowable. At stall it is 2.3 kN. The stop depends on a drive-side torque limit in a protocol nobody has read. Use steel pins.
- **Accuracy** (`rnd/accuracy.py:44`): any joint without a belt is given 0.05° "as a DM-J4310". The re-run shows the roll at 0.05° even without joint encoders. A two-stage 36:1 planetary typically has 10–20 arcmin of backlash, which is 2–3.5 mm at 0.6 m, and the roll torque crosses zero in the arm-down pose. The 0.9 mm headline excludes it. (S on the backlash figure.)
- **Ask the seller** for backlash, the output-bearing rating and which face is the output. The belt roll stays the safer M1 choice until then.

### M2. Bench power and safety electronics are not buildable or adequate as specified (V)

- **Supply:** the kit buys a 10 A CC supply and a 15 A fuse. `rnd/electrical.py:25-31` gives the arm branch 11.4 A sustained and 33.3 A peak. Astra finding 16 (15 A against 33 A) got worse, not better. A current-limit foldback on 4.7 mF browns the bus out in milliseconds, and the fuse can never blow.
- **Pre-charge against discharge:** the clamp is "forced on while the contactor is open", and pre-charge runs through 3.3 Ω while the contactor is open. As written the bus sits near 2 V and K_PRE times out. No interlock is described.
- **No schematic:** the clamp, watchdog and pre-charge exist as prose plus "comparator/reference, gate driver". This is the mandatory safety board, handling 97 A pulses, and a first-time builder would be designing it.
- **Watchdog margin:** an 8 ms hardware watchdog on a Python loop on non-real-time Linux over USB-CAN will trip on ordinary jitter, and each trip is a category-0 drop. (S) The 50 ms budget leaves room for about 25 ms.
- **Fix:** after the 1 A first power-up, use a 24 V supply of 25 A or more. Draw and review a schematic. Gate the forced discharge as a one-shot on the opening edge. Widen the watchdog.

### M3. Joint encoders and the wrist servos have no mount and no software (V)

- No encoder, magnet or board pocket exists in any arm CAD file (a search of `rnd/parts/*.py` finds only the tactile pad).
- The shaft bores an on-axis AS5048A needs are taken by cables, the balancer crank and the hub.
- `sw/bringup.py:31` says the RP2040 nodes and the STS3215 bus through the elbow node are not covered. There is no node firmware in the tree.
- **Consequence:** M1's stated proofs ("hand accuracy with the joint encoders ≤ 1 mm", "the fin-ray grasp") cannot run, and the wrist roll and gripper cannot move through the planned path.
- **Fix:** design the mounts (off-axis ring magnets or through-bore sensors) and write the node firmware first, or drive the servos from the USB-TTL adapter for M1 and drop the accuracy claim.

### M4. The GIM protocol gates everything; three driver types in one arm (S)

- `sw/drivers/gim.py` is all TODO. The safety concept (total-torque bound, torque feedback for collision detection, CAN timeout, a flash torque limit) assumes an MIT-style impedance mode on the SDC104, the SDC303 and the GIM8108-8's driver.
- If one of them offers only position or velocity modes, `sw/safety.py` does not apply to that joint.
- **Fix:** buy one GIM6010-8, prove enable, impedance, limit and timeout on it, then buy the rest.

### M5. Belt adjusters are used up at nominal; drive joints rely on friction (V geometry, S capacities)

- I solved the centre distance for the stock belts against the ±1.0 mm eccentric (`rnd/parts/belt_stage.py:108`):

| Stage | Stock belt | Centre in CAD | Centre needed | Adjuster used |
|---|---|---|---|---|
| Yaw | 315 mm | 80.00 (`roll.py:57`) | 80.94 | 94 % |
| Elbow | 295 mm | 70.00 | 70.71 | 71 % |
| Pitch | 355 mm | 70.00 | 70.29 | 29 % |

- The yaw stage has nothing left for tension, print tolerance or belt tolerance. Astra finding 11 is not closed. Fix: set the centres to 80.9 and 70.7.
- Torque passes through an aluminium pulley with set screws on a Ø12 flat, and through a clamp hub on a plain shaft. At the 11 N·m peak that is 1.8 kN at the shaft surface, reversing. Generic D40 Oldham couplings are catalogued at about 9–10 N·m. Fix: flats on both seats, retaining compound, and a datasheet rating of at least 12 N·m.

### M6. Hold A is not "motors only" (V)

- The buy-now list has 3 × D40 Oldham couplings for pitch, yaw and elbow. The D20 variant needs Ø50 couplings at pitch and elbow (`cartridge_m.py:27-28`). The M1 doc says "no other bought part changes".
- With 22 N·m at the 20T pulley, the repo's own check gives 1382 N peak pull and 2.8× ultimate (flagged ⚠️ by `PIP_ARM_MOTORS=d20 python -m rnd.belts`).
- That pull is about 0.7 kN on each 6801 (C0 about 1.0 kN), and 3.7 kN at the Ø12 clamp and set screws.
- **Fix:** D20 needs a Ø15 keyed stage and a 25 mm or steel-cord belt. Treat it as a redesign.

### M7. Thermal and torque margins are optimistic (V assumptions)

- `rnd/thermal.py:23-30`: rated torque is taken to give 60 K in still air, and the enclosure adds a flat 10 K. The motors sit in closed 3 mm ASA sleeves with a plastic cap and plastic couplings, so there is no conduction path. R17 then adds covers.
- The rated margin is 1.55× only at the 95th percentile. At the worst pose it is 1.17× (pitch) and 1.36× (yaw, elbow).
- The scorecard's planar "2.21×" row scales every mass inward with reach (`rnd/arm_sizing.py:24-26`), which is an artefact.
- Expect derating. Fit thermistors and a metal rear cap from day 1; neither is in the kit.

### M8. Balancer pulleys overloaded (S)

- The moving pulley carries the spring force, 634–748 N, on a U604 (a 604 bearing, C0 about 0.35 kN).
- The Ø11 sheave on 2 mm UHMWPE cord is a D/d of 5.5.
- **Fix:** use 608- or 625-class grooved pulleys (`rnd/balancer.py:31`).

## Minor

- **R17 is not on the scorecard** although `pinch.md` reports 62 nip fails and 31 exposed drive parts. The bench arm is not finger-safe: keep the keep-out zone.
- **`M1_READINESS.md` is stale in places:** "CAD assumes Ø100 × 60 and 6 × M4", "owner OK pending". `stiffness.md` (8.5 mm), the scorecard (11.0) and the readiness doc (10.9) disagree.
- **Custom metal parts are not in the kit:** the balancer crank (Ø44 × 4 aluminium with a pressed Ø6 pin) and a stand. The 7075 roll adapter is listed, at an estimated $30.
- **Six countersunk screws plus a centring lip** over-constrain the flange. Use button heads.
- **The regen and branch tables still assume four GIM6010s** on the roll row.
- **DM-J4310 `t_max` as a torque limit** is the MIT mapping range. The code marks it VERIFY, which is right.

## Earlier reviews: what really closed

- **Closed:** trapped bearings and split pulleys (D13, D19), the balancer (real spring, sign fixed), clock-spring packing, regen energy sizing, supervisor v2 torque bound, the roll teeth.
- **Not closed:**
  - Astra 2: now C1.
  - Astra 11: the yaw belt, M5.
  - Astra 14: R14 is nominal only, now C3.
  - Astra 16 and 18: now M2.
  - Astra 9 and Gemini 7: margins and thermal, M7.
  - Astra 22: the commissioning items.

## What's solid

- **The process.** ESTIMATE tags, holds on unconfirmed motors, seller questions, coupons before bulk buying, and a current-limited first power-up with a catch and tether.
- **The drive train for the GIM6010 set.** Steel shafts, bought pulleys, sustained belt pull 1.5–1.8× on a conservative 312 N rating, one-piece shaft-pulleys, and the bearing slide-on check.
- **The software core.** The MIT codec matches Damiao's layout. The supervisor's total-torque bound, latched faults, and stale or NaN feedback checks are sound. The momentum-observer math is correct.
- **Honest negatives.** The P2 FEA rerun, R2b reported as failing, wrist roll limited to ±120°, and R17 measured rather than assumed.
- **Buy without regret:** the DM-J4310, the STS3215s, one GIM6010-8, bearings, belts, CAN adapters, the contactor and mushrooms, and filament.
