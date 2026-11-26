# Confidence assessment, evidence-based (confidence campaign, 2026-11-25)

"Confidence" here means **the chance that the area works on the first physical build without a redesign** (software aside).
Each number moves only when new evidence arrives; the evidence is linked.

## What was run

| Check | Result | Report |
|---|---|---|
| Monte Carlo of the dynamic fetch task, 100 runs. Per run: torque derate 0.75–1.0, arm mass ×1.0–1.25, joint friction 0–0.6 N·m, calibration σ 0.1°, finger μ 0.5–1.2, can 0.2–0.6 kg, ±15 mm can offset | **100/100 succeed (95 % lower bound 96 %)**; worst actuator use 47 % of derated peak; tip margin ≥ 68 mm in every run | [mc.md](mc.md) |
| FEA (gmsh + scikit-fem, linear tetrahedral) of 7 critical printed parts at worst loads, vs a 12 MPa across-layer allowable | margins 3.5–53× after the fixes below | [fea.md](fea.md) |
| Motor thermal model: non-stop task day in the MC worst corner, and worst-pose holds | hottest motor 55 °C (limit 100 °C); every joint holds its worst pose indefinitely | [thermal.md](thermal.md) |
| **New load-path integration check:** torque pins, journals, clashes inside modules | **found real gaps (below)** | [integration.md](integration.md) |

## What the campaign found (all invisible to the earlier sims)

| Finding | Status |
|---|---|
| The elbow small-pulley journals were 30 mm from the housing walls, so belt tension loaded the 1.2 mm shell. FEA: 28 MPa (2.4× over), 14 mm deflection | **Fixed:** elbow belt cassette (2 plates on their own thin 6708 bearings, loop closed inside, pin in a radial slot) + screw-on bone cap. Cassette plate FEA 11×; housing now 4.6× |
| The elbow 40T pulley was 9.4 mm out of the belt plane | **Fixed** (`BELT_Y`) |
| Roll and yaw stages: hard-coded journal constants predated the taller print-in-place hub. Walls cut 2–3 mm into the KX hub and pulley (≈ 20 cm³ of clashes); belts 2.6 mm out of plane | **Fixed:** constants derived from the hub geometry |
| Clevis bottom yaw plate was a free solid (no structural tie) | **Fixed:** 2 spacer posts outside the belt loop |
| **Cartridge torque reaction: 18 of 19 torque pins end in air.** No stage has the slot the pin should engage | **Open, queue item 1** (one standard bay-ring/strap design for all stages) |
| Shoulder pitch: same stale-hub issue (10.4 cm³) | Open, queue item 2 |
| Elbow KX collar vs housing corner (1.8 cm³); forearm disc vs elbow boss (2.7 cm³, rotating) | Open, queue item 3 |
| Monte Carlo bug: rescaling masses left MuJoCo's cached subtree mass stale, so tip margins read as −10 mm | Fixed in `sim.apply_perturb` |

## Confidence by area

| Area | Before | Now | Evidence that moved it / what still caps it |
|---|---|---|---|
| Task-level actuator sizing and control authority | ~75 % | **90 %** | 100/100 MC with heavier arms, weaker motors, friction. Cap: real belt compliance, cogging |
| Mass, stability, tip-over | ~85 % | **92 %** | tip ≥ 68 mm in all MC runs, dynamic ZMP. Cap: real mass vs CAD (MC covers +25 %) |
| Printed-part strength | ~80 % | **85 %** | FEA 3.5–53× on the 7 worst parts. Cap: layer adhesion is an ESTIMATE; the torque-slot parts don't exist yet |
| Motors / thermal / electrical | ~75 % | **85 %** | thermal model 55 °C worst. Cap: ΔT-at-rated and τ are ESTIMATEs until the bench arm logs a motor |
| Belts and pulleys | ~65 % | **75 %** | three misalignments and the unsupported elbow hub fixed; supports FEA-checked. Cap: printed tooth form (coupon C6) |
| CAD integration completeness | not rated | **60 %, ~85 % once queue items 1–3 close** | new check found the torque-path gap on every stage |
| Stiffness / accuracy | ~65 % | ~65 % | housing deflects 1.3 mm at 1.5× load; joint encoders don't see link deflection. Needs a stiffness sim with the CF tubes in the loop |
| Lift | ~70 % | ~70 % | no new evidence (coupon C8 ratchet) |
| Balancer | ~70 % | ~70 % | no new evidence |
| Base / carpet | ~60 % | ~60 % | threshold/carpet Monte Carlo still queued |
| KX couplings (preload, slip, repeatability) | ~50–60 % | ~55–60 % | collar FEA 8.7×. **Simulation can't settle this:** print tolerance, friction and creep need coupons C1–C4 |
| **Overall: works on the first full build** | 50–60 % | **≈ 55 % today, ≈ 65–70 % after queue items 1–3** | the fixes remove failures that would have happened at assembly |
| **Overall: staged build (coupons → M1 bench arm → full robot)** | ~85 % | **≈ 90 %** | each stage catches the remaining unknowns cheaply |

## How to push the numbers higher

1. **Close queue items 1–3** (CAD, no cost). The hourly loop does this next: +10–15 points on "first build".
2. **Coupon round C1** ([COUPONS.md](../COUPONS.md)): ~$25 of parts and ~7.5 h of P2S printing. It measures the five ESTIMATEs that simulation can't reach:
    - clearance
    - press fit
    - slip torque
    - belt engagement
    - ratchet creep

    KX couplings would move to ~80 %, belts to ~85 %, lift to ~80 %.
3. **M1 bench arm** (≈ $1,100): real motor temperatures, stiffness and repeatability. That's the step from ~70 % to ~90 % on the full robot.
