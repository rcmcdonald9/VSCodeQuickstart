### Engineering Design Review — Pip v4

This review evaluates the Pip v4 hardware packet independently against mechanical load paths, FDM manufacturability, mechatronics, thermal physics, and home-robot safety.

---

### Findings

1. **Un-assemblable monolithic shafts and trapped bearings across all belt stages and wheel pods**
   - **Severity:** Critical
   - **Evidence:** `rnd/parts/shoulder.py` (`pitch_shaft()`, `bracket()`), `rnd/parts/roll.py` (`roll_shaft()`, `roll_link()`, `yaw_shaft()`, `clevis_and_yaw_block()`), `rnd/parts/arm.py` (`elbow_pulley_hub()`, `upper_bone()`), `rnd/parts/belt_stage.py` (`small_hub()`), `rnd/parts/base.py` (`wheel_shaft()`, `hub_pip()`, `pod()`).
   - **Why it matters physically:** Every joint stage unions oversized pulleys or flanges into a single solid shaft inside a monolithic dual-wall bracket:
     - `small_hub()` fuses an Ø87 mm KX74 hub, Ø20 mm top journal, Ø35.8 mm flanged pulley, and Ø20 mm bottom journal into one print—the top 61804 bearing (ID 20 mm) cannot slide over either end.
     - `pitch_shaft()` fuses a Ø95.5 mm 60T pulley between the two fixed walls of `bracket()` (Ø52 mm bearing seats) with a Ø91 mm ferrule on the end; `roll_shaft()` fuses a Ø114.6 mm 72T pulley inside monolithic `roll_link()`. Neither can be inserted.
     - `elbow_pulley_hub()` is 132 mm long (`HOUSING["y"] + 24`) with an integral Ø63.7 mm pulley trying to enter a 105.6 mm interior housing width.
     - `wheel_shaft()` (Ø60 mm flange) and `hub_pip` (Ø87 mm hub) trap two 61808 bearings between them with no bolted joint between hub and shaft.
     - In addition, torque transfer from `roll_shaft()` into `clevis_and_yaw_block()` (41.8 N·m peak into smooth Ø40.4 mm holes) and from `elbow_pulley_hub()` into `forearm()` (Ø40 mm shaft in Ø42 mm hole) has no keyway, spline, clamp split, or bolt circle.
   - **Fix / test:** Split every driven shaft into a through-shaft/hub and a separately bolted pulley/flange (or split bearing caps with M4 cap screws) so bearings slide onto unobstructed cylindrical journals, and add positive torsional coupling (printed/steel keyway, polygon/D-flat, or bolted flange with ≥4×M4) at every shaft-to-link interface. Add an automated assembly-disassembly clearance check (`can_slide_along_axis`) to `rnd/integration.py`.
   - **Confidence:** 100%

2. **KX coupling spring pocket, ball retention, and print-in-place geometry are physically impossible as modelled**
   - **Severity:** Critical
   - **Evidence:** `rnd/parts/kx.py` (`KX`, `HUB_STEP = 6.0`, `STEP_IN = 1.5`, `LIP_T = 2.0`, `spring_t = 1.2`, `stroke = 1.0`, `male()`, `hub_pip()`, `collar()`), `docs/rnd/kx_mc.md` (`compression c ≈ 1.5 mm nominal`), `docs/rnd/BOM_v4.md` (`6 small coil springs + PTFE washer per socket`), `docs/COUPONS.md` (C1, C2).
   - **Why it matters physically:**
     - **Spring cavity cannot fit any spring:** `STEP_IN = 1.5 mm` minus 0.5 mm radial clearances leaves an annular pocket only **1.0 mm wide radially** and **1.2 mm tall axially** (`spring_t = 1.2`). When the collar lifts by `stroke = 1.0 mm` (or `1.5 mm` in `kx_mc.md`), only **0.2 mm** (or negative clearance!) remains for the PTFE washer and 6 coil springs—coil bind will occur before the collar even reaches the ramp.
     - **Balls will fall out of `male()`:** The ball centre sits at `z = 0`, and the pin relief cut removes flange material up to `z = +(pin_proud + 0.5) = +1.1 mm`. For KX74 (Ø10 mm ball), only **3.9 mm (<39% of diameter)** is embedded in PETG, leaving zero undercut. Printing a pause to "embed" the balls cannot capture them, and tangential slip loads (146 N/ball) pry them straight out of shallow spherical caps.
     - **Unprintable `hub_pip`:** In `hub_pip()`, the collar bottom (`z = -9.2`) floats **1.0 mm in mid-air** above the retainer ring (`z = -10.2`), and the two print pauses described in `COUPONS.md` are ordered upside down relative to Z (`pin_z = -1.9` vs `collar lip = -7.2`).
   - **Fix / test:** Abandon print-in-place sockets. Use a screwed retainer ring (3×M3 into heat-set inserts—only +1 min/socket with a power driver) with a proper **6–8 mm tall, ≥6 mm wide** spring pocket housing 6× Ø5 mm die/coil springs (wire Ø0.8–1.0 mm, free length ~10 mm, 2–3 mm compression so tolerance sensitivity drops 2×) under a 0.5 mm PTFE thrust washer. Recess the ball centres 1.0–1.5 mm *into* the male flange (or retain them with a thin laser-cut/printed keeper plate) so >60% of the ball is captured. Print and cycle coupon C1/C2 immediately.
   - **Confidence:** 98%

3. **KX PETG creep at 55 °C and tooth bending stress exceed allowables; 10-year creep model is over-optimistic**
   - **Severity:** Major
   - **Evidence:** `docs/rnd/kx.md` (tooth bending: KX74 14 MPa, KX56 17 MPa, KX44 12 MPa), `docs/rnd/fea.md` (`12 MPa` allowable, KX74 collar 99.5th pct 1.4 MPa vs max 9.4 MPa), `docs/rnd/kx_mc.md` (creep modulus ratio `U(0.30, 0.40)` at 55 °C, 7.5% mean preload loss, `P(slip < peak) = 25.6%`).
   - **Why it matters physically:** PETG has a glass transition $T_g \approx 75\text{–}80\text{ °C}$ and HDT (0.45 MPa) $\approx 65\text{–}70\text{ °C}$. Holding PETG lugs and collar teeth continuously at 12–17 MPa bending stress (already at or above the 12 MPa static allowable!) while bolted to warm motor housings (55–70 °C) drives nonlinear viscoplastic creep and rapid stress relaxation over days/weeks—far worse than the assumed 7.5% linear-elastic loss over 10 years. Worse, `P(slip < peak) = 25.6%` means a quarter of joints (including drive wheels hitting thresholds at 11 N·m!) will slip and lose zero during normal transient peaks.
   - **Fix / test:** Increase `tooth_t` and `lug_t` from 2.2–3.0 mm to 4.0–5.0 mm with root fillets ≥1.5 mm, increase spring stroke to 2.5–3.0 mm (lower spring rate $k$ means plastic creep of 0.15 mm loses 5% preload instead of 15%), and print KX collars/males in **PETG-CF, PA6-CF, or PC-blend** ($T_g > 100\text{ °C}$). Run coupon C1's 50 °C creep test at **65 °C for 168 h** and measure actual torque retention.
   - **Confidence:** 90%

4. **Clock-spring cable cavities do not exist in CAD, wrist FFC length violates MIPI-CSI signal integrity, and main lift/head cables are missing**
   - **Severity:** Critical
   - **Evidence:** `docs/rnd/harness.md` (`6/6 joints ✅`, cavities `r 22–60` pitch, `22–50` roll/elbow, `16–30` wrist pitch, `14–24` wrist roll, yaw free length `240 mm`, wrist FFC `0.35 + 2×0.3 = 0.95 m`), `rnd/parts/arm.py`, `rnd/parts/roll.py`, `rnd/parts/shoulder.py`, `rnd/parts/hand.py`, `rnd/parts/upper.py`.
   - **Why it matters physically:**
     - **Zero CAD volume for spirals:** None of the 5 clock-spring cavities (`Ø48–120 mm` × 8–10 mm axial space) exist in CAD; bearing housings, pulleys, balancer levers, and clevis arms occupy that exact space. Wrist pitch (`DM-J4310`) and wrist roll (`STS3215`) have solid shafts with no through-bores.
     - **Yaw free length is ~145 mm, not 240 mm:** From the top of `yaw_shaft` (`z = -56` in roll frame, only ~2 mm below the spinning 72T roll pulley!) down to the elbow small pulley (`z = -103` in arm frame) leaves ~145 mm max before hitting the elbow cassette—insufficient for ±70° torsion (needs 194 mm).
     - **0.95 m unshielded MIPI-CSI FFC:** Standard CSI-2 FFCs fail past ~200–300 mm, especially coiled inside spiral cavities beside 12 V/TTL servo PWM lines.
     - **Omitted moving harnesses:** `harness.md` omits the **USB3 SuperSpeed cable** for the head Gemini 2 camera, omits the **entire base-to-torso moving cable chain** (35 A 24 V main power, CAN A, lidar USB, e-stop loop across 0.70 m lift travel!), and `electrical.md` omits USB from the arm detach connector.
   - **Fix / test:** Switch wrist cameras to compact global-shutter **USB2 UVC** modules (OV9281 UVC) over a single slim Ø2.2 mm shielded silicone 4-core cable (eliminating the CSI→USB forearm bridge and 0.95 m FFC). Explicitly model every clock-spring cassette (adding axial stack height and re-checking arm mass/inertia), route a real drag chain along the mast for base↔torso and torso↔head (including USB3), and route wrist cables via an external service loop around the wrist roll.
   - **Confidence:** 95%

5. **Wrist pitch, wrist roll, and head tilt violate D3 ("cartridges carry torque only") and overload motor bearings/servos**
   - **Severity:** Major
   - **Evidence:** `rnd/parts/arm.py` (`forearm()`), `rnd/parts/hand.py` (`palm()`, `gripper_parts()`), `rnd/parts/upper.py` (`head()`), `docs/rnd/kx.md` (claims `2×61805` on wrist pitch and wrist roll), `docs/RND_PLAN.md` (D3).
   - **Why it matters physically:** Contrary to `kx.md` and D3, CAD models **no 61805 bearings** at wrist pitch or wrist roll—the palm and gripper hang directly off the DM-J4310 and STS3215 output shafts via KX56/KX44 hubs. The STS3215's tiny output bushing/brass horn cannot sustain 1.2 N·m cantilevered bending + crash loads without severe play or stripping. Similarly, in `upper.py`, the 0.94 kg head (COM ~80 mm forward of the tilt axis) is cantilevered off a **single STS3215 horn** on one side of the yoke with no opposite pivot bearing, consuming ~0.74 N·m (**75% of rated 0.98 N·m**) continuously just to hold the head level.
   - **Fix / test:** Add the specified 61805 bearing pairs to the forearm wrist housing and palm, or drop KX44 at the wrist roll/gripper to save 30 mm of hand length and mount a proper crossed-roller/dual-6805 yoke. On the head, move the tilt axis forward under the head COM (`x ≈ +60 mm`, cutting holding torque to <0.15 N·m) and add a passive 625 pivot bearing on the `-Y` yoke arm.
   - **Confidence:** 98%

6. **E-stop architecture has a 0.5 s unsafe delay, lacks regenerative braking protection and pre-charge, and leaves 12 V servos un-isolated**
   - **Severity:** Critical
   - **Evidence:** `docs/rnd/electrical.md` (`E-stop behaviour`, `Power distribution`, `Pack peak 69 A`), `docs/rnd/BOM_v4.md` (`E-stop: 2 latching mushrooms + 24 V contactor + hold-up capacitor $35`).
   - **Why it matters physically:**
     - **0.5 s capacitor hold-up on an e-stop coil** delays hardware power cutoff for half a second during an emergency (enough time for a runaway arm to strike a person or crush a finger) and introduces a single-point failure mode if the capacitor shorts.
     - **No regen clamp / dump resistor:** Opening the main contactor while 15 BLDC drivers are moving or an arm is falling traps regenerative energy on the downstream 24 V bus; bus voltage spikes above 36–48 V will destroy the GIM6010-8 SDC104 and DM-J4310 drivers.
     - **No pre-charge circuit:** Closing the contactor directly onto 15 motor controllers' bulk capacitance causes a 100–300 A inrush spike that trips the 100 A BMS or welds cheap relay contacts.
     - **12 V servo branch bypasses the main contactor** (relying on an unspecified, un-BOM'd "servo relay").
   - **Fix / test:** Remove the coil hold-up capacitor (use immediate Category 0 STO power drop on e-stop), add a **TVS + active shunt brake resistor (~28 V clamp, 50 W pulse)** downstream of the contactor, add a **5–10 Ω pre-charge resistor** with a 1 s bypass relay (or solid-state high-side smart FET switch like TI TPS12110 / BTS50010), use a real DC-rated contactor (e.g., Albright SW60 or dual back-to-back 100 A MOSFET e-stop switch), and feed the 12 V servo DC-DC from *downstream* of the e-stop.
   - **Confidence:** 95%

7. **Motor thermal model ignores enclosed PETG housings and PETG's ~60 °C structural softening limit**
   - **Severity:** Critical
   - **Evidence:** `docs/rnd/thermal.md` (`Ambient 30 °C, limit 100 °C, ΔT at rated 60 K, still air`, `shoulder_roll` worst static hold `75 °C`, `shoulder_pitch` `70 °C`), `rnd/parts/cartridge_m.py` (3 mm PETG can), `rnd/parts/upper.py` (`torso_shell`, `lift_cover`), `rnd/parts/roll.py` (`shoulder_cap`).
   - **Why it matters physically:** Wrapping a BLDC motor + integrated MOSFET driver in a 3 mm PETG sleeve (`k ≈ 0.2 W/m·K`), then enclosing it inside sealed cosmetic shells (`torso_shell`, `shoulder_cap`, `upper_bone`, `lift_cover`) with zero airflow easily doubles or triples the thermal resistance to ambient. Even under `thermal.md`'s optimistic open-air assumptions, `shoulder_roll` reaches 75 °C and `shoulder_pitch` reaches 70 °C—right at **PETG's HDT (65–70 °C)**! The motor cans, 3×M3 rear mounting floors, and KX74 hubs will soften, warp, and lose preload long before the copper windings reach 100 °C.
   - **Fix / test:** Cap allowable **motor case temperature at ≤55 °C** (not 100 °C winding), add ventilation louvers + a 40 mm exhaust fan in `torso_shell` (also required for the Pi 5), open the cosmetic caps around the motor rear faces so the aluminum motor housing radiates/convects directly to room air (use an aluminum mounting plate between motor and PETG can), and log actual GIM6010-8 case and driver temperatures inside a printed PETG can at 5 N·m continuous on the bench before building M1.
   - **Confidence:** 92%

8. **Roll belt stage CAD/BOM mismatch (20:72 vs 18:72), non-HTD printed tooth profile, and un-checked interlayer shear on `small_hub`**
   - **Severity:** Critical
   - **Evidence:** `rnd/parts/roll.py` line 32 (`small_hub()` defaults to 20T) vs `docs/rnd/belts.md` (`18:72, 426 mm`) and `docs/rnd/BOM_v4.md` (`425 mm`), `rnd/parts/belt_stage.py` (`pulley()`, `small_hub()`, `eccentric_bushing()`), `docs/rnd/fea.md`.
   - **Why it matters physically:**
     - **Belt won't fit CAD:** `roll.py` builds a 20T small pulley with the 72T big pulley at 90 mm centre distance, requiring a **429.0 mm** belt—4 mm longer than the 425 mm BOM belt, exceeding the ±2 mm belt-length adjustment of `eccentric_bushing()` (`e = 1.0 mm`).
     - **Interlayer failure of `small_hub`:** Never FEA'd. The Ø20 mm (ID 8.4 mm) printed PETG journal carries 12.7 N·m KX slip torque (**8.34 MPa pure interlayer shear**) simultaneously with 768 N roll belt bending (~3.2 MPa nominal, **~8 MPa with sharp-step SCF**) across Z-layers -> combined **~16.5 MPa von Mises across layers** (>12 MPa allowable!).
     - **Straight-walled tooth form & dual-eccentric skew:** `pulley()` cuts a straight-sided U-slot (`Box(1.6, 3.2)`) rather than a true HTD curvilinear profile, risking chordal climb and tooth jump on only 6.1 teeth in mesh (18T). Two independent eccentric bushings per shaft (wall thickness only 2.0 mm at the thin side under 384 N peak!) can easily skew the shaft axis and bind the 61804 bearings, and constant belt tension will creep the PETG walls/bushings and slacken the belt.
   - **Fix / test:** Pass `teeth=18` explicitly in `roll.py` (or better, standardize on **off-the-shelf aluminum 20T/18T HTD 5M pulleys**, ~$4–$6 each, pinned onto a **steel Ø10–12 mm or Ø20 mm Al stub shaft**!). Keep printed pulleys only for the large 40T–72T pulleys using true HTD 5M DXF/spline profiles, and replace dual eccentrics with a single rigid bearing-cartridge slide or spring-loaded idler tensioner. Test coupon C6 under **full 768 N peak torque** (not just 300 N hand load!).
   - **Confidence:** 95%

9. **FEA methodology masks real stress peaks (99.5th pct truncation + P1 tet shear locking) and omits structural compliance from accuracy claims**
   - **Severity:** Major
   - **Evidence:** `docs/rnd/fea.md` (`linear tetrahedral`, `99.5th percentile`, `upper_bone` 2.7 MPa vs max 17.5 MPa, disp 1.91 mm; `clevis + yaw block` disp 1.90 mm; `KX74 collar` 1.4 MPa vs max 9.4 MPa), `docs/rnd/SCORECARD.md` (`hand sag 1.8 mm ✅`, `hand error 0.9 mm ✅`).
   - **Why it matters physically:**
     - Discarding the top 0.5% of 65k–95k elements drops **325–480 elements**, hiding real stress concentrations across entire tooth roots and bosses (e.g., `upper_bone` max 17.5 MPa > 12 MPa).
     - First-order (P1) linear tetrahedra across 1.2 mm thin walls (`WALL = 1.2 mm`) suffer severe bending shear locking, under-predicting deflections.
     - Even *with* stiff P1 tets, `clevis + yaw block` deflects 1.90 mm and `upper_bone` deflects 1.91 mm individually; angular deflection at the shoulder/yaw amplified by a 0.65 m lever arm plus mast sway (1.5 mm), KX compliance, V-band flange compliance, and belt stretch will produce **8–20 mm of static hand sag** at 1 kg—invalidating both the `1.8 mm` sag and `0.9 mm` positioning accuracy Scorecard passes (joint encoders cannot observe structural or belt-output deflection).
     - Epoxy bonding of roll-wrapped CF tubes into smooth PETG sockets (`arm.py`) has poor peel/shear strength because PETG has low surface energy.
   - **Fix / test:** Re-run FEA with **quadratic (P2) tetrahedra**, report peak stress at filleted regions (not global 99.5th pct), add FEA for `carriage`, `mast coupling`, `pitch_shaft`, and all torque forks, and perform a full-chain static compliance FEA from base to hand. For CF-to-PETG joints, scuff + flame/isopropanol prep + use **methyl methacrylate (MMA) structural adhesive** AND add **2×M3 cross-bolts/pins** through each socket as a mechanical fail-safe against torsional debonding.
   - **Confidence:** 90%

10. **Shoulder gravity balancer spring cannot physically fit in its 135 mm cage, and unlatched arm removal creates a snap hazard**
    - **Severity:** Major
    - **Evidence:** `rnd/parts/shoulder.py` (`cage` length 135 mm, ID 24.8 mm, `pulley_a` flat Ø16 mm cylinder at `z = 50`, pin B at `r = 30`), `docs/rnd/spring.md` (`K = 4.5 N·m`), `docs/RND_PLAN.md` (D4: `2.7 N/mm` compression spring), `docs/rnd/harness.md` (pitch range `260°`).
    - **Why it matters physically:** For a zero-free-length balancer ($\tau = K \sin\theta$) with $a = 50\text{–}65\text{ mm}$ and $b = 30\text{ mm}$, the cable extension at arm-down ($\theta = 0^\circ$) is $a + b = 80\text{–}95\text{ mm}$ (working stroke 60 mm, peak force 216–256 N). A compression spring capable of 80–95 mm deflection without coil-bind requires a **170–220 mm free length**—it will not fit inside the 135 mm PETG tube (and the 1.6 mm PETG cage wall carrying 256 N axial compression + side pull will buckle/creep). Furthermore, `pulley_a` has no groove (`Cylinder(8, 6)`), its Ø16 mm diameter is too tight for 2 mm UHMWPE cord under 220 N (causing rapid fiber fatigue), and detaching the 3.9 kg arm while the spring is loaded snaps the bare pitch ferrule upward.
    - **Fix / test:** Replace the single long compression spring with a **dual-extension-spring or gas-strut/clock-spring balancer**, or route the compression spring vertically down the torso spine/bracket (giving ≥220 mm length) with a **grooved ball-bearing pulley (≥Ø24 mm)** and an aluminum spring tube. Add a captive **park-lock pin** on the pitch bracket that engages before releasing the V-band clamp.
    - **Confidence:** 93%

11. **Base tip-over margin is dangerously thin for two-handed loads, off-axis reaches, thresholds, and accidental human bumps**
    - **Severity:** Major
    - **Evidence:** `docs/rnd/twin.md` (`X wheel-fwd R.21` min static margin `64 mm`, `X edge-fwd` `50 mm`), `docs/rnd/SCORECARD.md` (`64 mm ✅`, `MC tip ≥ 68 mm`), `docs/rnd/threshold_mc.md` (tilt up to `3.7°`, peak torque `11.0 N·m`), `rnd/params.py` (`R_W = 0.21 m`, `head_mass = 0.94 kg` at `1.29 m`).
    - **Why it matters physically:** A 4-wheel `+` layout at $R = 0.21\text{ m}$ has flat support edges only **148.5 mm** from centre (at ±45°). `twin.md` only tests a single 1 kg arm reaching straight over the front vertex (210 mm), and `mc.md` only tests a 0.2–0.6 kg can on a flat floor. It never checks **R2's 2.5 kg two-handed payload** with both 3.9 kg arms extended, nor reaching at ±45° over the 148.5 mm flat edge. At `COM z ≈ 0.65 m`, a `3.7°` tilt crossing a 14.5 mm threshold shifts COM by **42 mm**, braking at `0.5 m/s²` shifts ZMP by **33 mm**, and an accidental **22 N (~2.2 kgf) nudge** at torso height tips the 23 kg, 1.29 m tall robot over onto a person or pet.
    - **Fix / test:** Add a **2.5 kg bimanual + 45°-azimuth + braking** tip-over test to `twin.py` and `mc.py`. Move the battery and base electronics as low and far back (`-X`) as possible (and consider moving the mast slightly further aft to `x = -80 mm` or widening `wheel_radius_pos` from `0.21 m` to `0.23 m` within the Ø500 mm shell). In firmware, enforce an active ZMP envelope monitor that slows lift/arm/base acceleration whenever estimated ZMP margin drops below 40 mm.
    - **Confidence:** 92%

12. **Threshold crossing hits 100% of motor peak torque (violating R4), risks KX wheel slip, and bottoms out the skirt on carpet**
    - **Severity:** Major
    - **Evidence:** `docs/rnd/threshold_mc.md` (at `h = 8.2 mm` and `14.5 mm`, peak τ = `11.0 N·m`, min torque margin = **`1.00×`** vs R4 target `≥ 1.5×`), `docs/rnd/SCORECARD.md` (reports `1.7× ✅` from nominal run only), `rnd/parts/base.py` (`clearance = 25 mm`, un-joined 4-piece 6 mm deck quarters).
    - **Why it matters physically:** The Scorecard obscures that the Monte Carlo already saturates the GIM6010-8 at 11.0 N·m on legal ≤15 mm thresholds (`1.00×` margin). At 11.0 N·m, `kx_mc.md` predicts a **25.6% probability of slipping the wheel's KX74 coupling**! Furthermore, narrow 6" DuraOmni rollers sink 5–10 mm into medium-pile carpet, cutting the 25 mm skirt clearance to 15–20 mm so a 15 mm threshold scrapes the 2 mm PETG skirt. Structurally, `base.py` splits the 6 mm PETG deck into 4 separate quarters with **no joiner plates**, leaving the 1.1 m mast socket (`coupling()`) cantilevered on a single 6 mm quarter-deck.
    - **Fix / test:** Raise skirt ground clearance to **35–40 mm** (except a compliant rubber bumper lip), eliminate KX couplings on the 4 drive wheels (bolt the wheel hubs directly to GIM outputs—nobody swaps wheel motors daily, and it eliminates wheel slip/backlash and saves 4 sockets!), and tie all 4 deck quarters together with a **2–3 mm aluminum or steel central chassis ring/cruciform plate** connecting all 4 wheel pods directly to the mast coupling.
    - **Confidence:** 90%

13. **Self-docking has a physical clash (`bumper_light`), relies on unrealistic frictionless omni-slide physics, and requires screwing into home floors**
    - **Severity:** Major
    - **Evidence:** `rnd/parts/base.py` (`bumper_light` at `R = 250..254 mm, z = 43..53 mm`; `DOCK_PADS` flush at `R = 250 mm, z = 40, 60, 100 mm`), `rnd/parts/dock.py` (`SEAT_X` for `R_BASE = 250 mm`, `floor tabs for 2 screws each`), `docs/rnd/dock_sim.md` (planar x/y slides, 5 N lateral limit).
    - **Why it matters physically:**
      - **CAD interference:** The `bumper_light` ring protrudes 4 mm outside the Ø500 mm skirt right between the ID pad (`z = 40`) and `+` pad (`z = 60`), blocking the robot from seating against the V-wings and contact tower.
      - **Dock sim physics is invalid:** Real 8:1 geared GIM6010-8 omni wheels have 8–16 N of static gearbox/cogging friction per axis (plus carpet drag), not frictionless MuJoCo slides. A 5 N lateral current limit will wedge the robot on one V-wall instead of self-centering.
      - **Home usability:** Screwing a printed V-block into hardwood/tile/carpet floors is unacceptable for a home robot; without screws, a 60 N push slides the dock across the floor.
    - **Fix / test:** Cut the `bumper_light` ring clear of the 45° dock-pad window (or recess it flush). Redesign the dock with a **low-profile baseplate ramp (3–5 mm thick) that the robot's front wheel rides onto** so the robot's own weight anchors the dock (no floor screws!), and re-run `dock_sim.py` with realistic **4-wheel omni kinematics + 0.5 N·m gearbox Coulomb friction** per wheel.
    - **Confidence:** 92%

14. **Battery BMS, autonomous charging balance drift, and blind-mate arm plug have electrical failure modes**
    - **Severity:** Major
    - **Evidence:** `docs/rnd/electrical.md` (pack worst sustained `32.6 A`, dock stops at `4.15 V/cell` from `BMS telemetry`, `ideal-diode`, arm plug `XT30 + 6-pin 2 mm pogo`), `docs/rnd/BOM_v4.md` (`6S 30 A BMS $15`, `Lumenier NAV 6S pack`, `reverse-polarity diode/fuse $10`).
    - **Why it matters physically:**
      - A **$15 AliExpress 30 A BMS** is undersized for the stated **32.6 A sustained** pack draw, has **no digital telemetry** to report cell voltages to the dock, and wiring an inline BMS onto an unprotected FPV Li-ion pack (`Lumenier NAV`) requires risky high-current DIY soldering on live 21700 cells.
      - Stopping dock charging at **4.15 V/cell** prevents passive top-balancing (which triggers at 4.18–4.20 V/cell), causing cells to drift out of balance over repeated dock cycles until OVP/UVP trips.
      - Using a bare **XT30** (20–40 N mating force, tight alignment) as a "blind-mate" connector beside a hand-squeezed sanitary tri-clamp risks bent pins; sharing a 2 mm pogo GND pin parallel to the XT30 motor GND creates a ground-loop burnout hazard if the XT30 breaks first.
    - **Fix / test:** Specify a **6S 50 A+ Smart BMS with active balancing and UART/CAN telemetry** (e.g., JK-B1A8S20P or JBD Smart BMS, ~$45–$60) or buy an enclosed commercial 24 V e-bike/mobility pack with integrated smart BMS. On the arm detach interface, use a **proper floating blind-mate hybrid connector** (e.g., Anderson Powerpole + recessed pogo block with longer first-mate/last-break GND pin, or a short manual umbilical plug with an XT30 + locking Molex/JST-GH plugged in after clamping the V-band—takes 5 seconds and eliminates blind-mate alignment risks!).
    - **Confidence:** 94%

15. **CAN bus loading (76% at 500 Hz) over SPI/USB on a Pi 5 risks control jitter and dropped frames**
    - **Severity:** Major
    - **Evidence:** `docs/rnd/electrical.md` (`CAN A 66% + 10% = 76%`, `CAN B/C 65% + 10% = 75%` at 500 Hz, Waveshare 2-CH SPI CAN HAT + USB-CAN), `docs/RND_PLAN.md` (D10: Pi 5 at `~73% of 4 cores ESTIMATE`).
    - **Why it matters physically:** Operating classic CAN 2.0B above 50–60% bus utilization leaves little margin for arbitration bursts, error retransmissions, or asynchronous D11 screen/sensor packets; combined with USB 1 ms polling intervals and non-RT Linux SPI interrupts on a heavily loaded Pi 5 (also ingesting 3 cameras + lidar), missed motor command deadlines can trigger actuator watchdog faults (sudden limp arms!).
    - **Fix / test:** Drop non-critical loops (base wheels, lift, sensor nodes) to **200–250 Hz** (cutting CAN bus load to ~35–40% with zero loss in task performance; arm control at 250 Hz is ample for home manipulation), or use **CAN FD (5 Mbps data phase)** if supported by the actuators/nodes. Bench-test worst-case latency and packet drops on the Pi 5 with all 3 cameras + lidar streaming simultaneously before finalizing the compute architecture.
    - **Confidence:** 85%

16. **Lift safety relies on a single printed PETG ratchet tooth bonded to anodized aluminum, with severe drop/pinch hazards**
    - **Severity:** Critical
    - **Evidence:** `rnd/parts/upper.py` (`ratchet_strip()` 4 bonded PETG segments, `6 × 14 mm`), `docs/COUPONS.md` (C8: `15 kg for 1 h`, `×1.2` static), `docs/BUILD_v4.md` (arm torque limit `rated × 1.8`, KX slip `1.16 × peak`).
    - **Why it matters physically:**
      - If the lift belt snaps or e-stop engages during a 0.25 m/s descent, a **15.6–18 kg** upper assembly drops up to 5 mm before slamming onto a single 5 mm pitch **printed PETG tooth** held to smooth anodized aluminum by glue. Dynamic shock load (500–800 N) can shear the tooth or peel the adhesive strip, dropping the torso like a guillotine onto hands or the base.
      - Open belt runs (elbow side window, roll pulley bottom, vertical lift belt) and high arm torque limits (`34.2 N·m` roll / `25.6 N·m` pitch in firmware; `48.5 N·m` KX slip) pose serious finger/hair entrapment and pinch hazards in a domestic environment.
    - **Fix / test:** Replace the bonded PETG ratchet with a **bolted steel/aluminum toothed rack** (or drill/bolt an off-the-shelf Mod 1.5 steel rack strip to the mast) and a **steel or CNC aluminum pawl**; alternatively use a **24 V fail-safe electromagnetic disc brake** on the lift shaft (proven industrial practice, 0 W holding, ~250 g, ~$25). Drop-test C8 dynamically (release 20 kg from 10 mm free-fall onto the pawl 10 times!). Fully enclose all belt stages with snap-fit guards so no fingers can reach any belt-pulley nip point.
    - **Confidence:** 94%

17. **FDM printability on a 256 mm P2S ignores massive overhang supports, bed keep-out zones, and realistic build time**
    - **Severity:** Major
    - **Evidence:** `docs/rnd/assembly.md` (`skirt quarter 124×250×250`, `deck quarter 6×246×246`, `head_shell 136×240×240`, `torso_shell 195×210×230`), `docs/BUILD_v4.md` (`21.7 h hands-on`, `281 h print`, `12.1 kg PETG`), `docs/rnd/BOM_v4.md` (`13.9 kg + 20%`).
    - **Why it matters physically:**
      - Parts at 246–250 mm sit right on the Bambu 256 mm bed edge/exclusion zone where thin curved PETG walls warp and lift.
      - Monolithic dual-wall brackets (`shoulder_bracket`, `roll_link`, `clevis + yaw block`), overhanging torque forks, and compound-filleted `soft_shell` domes (`re = 38–40 mm`) require heavy supports in every orientation, increasing print time/waste by 25–35% and degrading bearing-seat tolerances.
      - The **21.7 h** hands-on estimate omits realistic wiring/clock-spring routing (8–15 h), fabricating 4 AnySkin magnet-elastomer tactile pads (4–6 h), aligning 9 AS5048A encoders (no CAD mounts exist yet!), and support removal/rework. Realistic first-build time is **45–70 h** plus 3–4 weeks of printer time.
    - **Fix / test:** Redesign brackets as **flat, bolt-together plates with aluminum/printed standoffs** (prints flat, zero supports, strongest in-plane XY fiber alignment, AND fixes Finding #1's trapped bearings!). Replace steep bottom dome fillets on `torso_shell`/`head_shell` with 45° chamfers + small fillets. Split the deck/skirt into 6 sectors (~175 mm) to stay comfortably inside the heated bed core. Revise `BUILD_v4.md` with realistic sub-assembly times and stop chasing an impossible 4 h target for a 21-DOF robot.
    - **Confidence:** 93%

18. **BOM cost ($3,903) leaves $97 headroom while omitting $600–$1,100 of real landed costs and missing components**
    - **Severity:** Major
    - **Evidence:** `docs/rnd/BOM_v4.md` (`$3,903` vs `$4,000` target; `E-stop $35`, `BMS $15`, `Connectors/wire $50`, `Wrist cam + CSI-USB bridge $55×2`, `Tactile pads $10×4`, `Fasteners/epoxy/grease $60`, zero shipping/tax/duties).
    - **Why it matters physically:** Multi-vendor shipping, import tariffs, and tax across 12–15 US/China suppliers will add ~$400–$700 alone. Under-budgeted items include a real DC e-stop contactor + pre-charge (+$70–$110), smart BMS (+$35), pre-made harness kit / high-flex cables / mast cable chain (+$150–$250), realistic CSI→USB bridges (+$100–$200 if kept), TPU filament (+$25), and missing CAD hardware (encoder mounts, deck joiner plates, remaining 17 torque hangers). Actual landed build cost will be **$4,500–$5,200**, breaching the $4,000 target.
    - **Fix / test:** Add a mandatory **15% shipping/tax/contingency line** to `rnd/bom.py`, price real part numbers for every `ESTIMATE` line over $20, drop the CSI→USB bridge in favor of direct OV9281 UVC USB2 cameras, and eliminate unnecessary KX sockets on the 4 drive wheels and lift (-5 KX sockets, -20% hardware/build time) to recover budget headroom.
    - **Confidence:** 92%

---

### 5 Assumptions Most Likely Wrong

1. **PETG preload loop loses only 7.5% preload over 10 years at 55 °C (`docs/rnd/kx_mc.md`).**
   Under 12–17 MPa continuous bending stress near 55–70 °C motors, FDM PETG will experience rapid nonlinear creep and stress relaxation—likely losing **25–50% of preload within days to weeks**, dropping KX74 slip torque below rated torque or causing chronic joint slippage.
2. **Motor temperatures stay ≤55 °C on a task day and ≤75 °C on worst-pose holds (`docs/rnd/thermal.md`).**
   Modeling `ΔT_rated = 60 K` in still open air ignores that motors are wrapped in 3 mm insulating PETG cans inside sealed plastic shells. Real case/driver temperatures under sustained loads will likely run **1.5×–2.5× higher** (hitting PETG's 65–70 °C softening point rapidly).
3. **Static hand sag is 1.8 mm and hand positioning error is 0.9 mm at 0.6 m (`docs/rnd/SCORECARD.md`).**
   Ignoring angular compliance of the printed shoulder bracket, clevis/yaw block (1.90 mm local disp in FEA!), V-band flange, KX ball seats, belt stretch, and P1-tet shear locking underestimates true hand deflection by an order of magnitude—real sag under 1 kg at reach will likely be **8–20 mm**.
4. **A 25 kg geared omni base will self-center passively in a 60° V-dock under a 5 N lateral limit (`docs/rnd/dock_sim.md`).**
   Modeling the base as frictionless planar x/y slides ignores 8–16 N of static planetary-gearbox friction per axis plus carpet roller scrub; the real base will bind/wedge asymmetrically on the first V-wall contact.
5. **The robot can be built for $3,903 in 22 hours hands-on (`docs/rnd/BOM_v4.md`, `docs/BUILD_v4.md`).**
   Once 12–15 vendor shipping/tariff costs, real DC contactor/smart-BMS/harness costs, support-heavy prints, encoder mounting, AnySkin fabrication, and debugging are counted, true cost will be **$4,500–$5,200** and first-build hands-on time **45–70 hours**.

---

### 5 Strongest Aspects of the Design

1. **Single-source-of-truth parametric pipeline (`rnd/params.py` → build123d CAD → MuJoCo twin → FEA → BOM → Scorecard).**
   Automating mass roll-ups, collision-hull export (`rnd/parts/collision.py`), clash checks, and regression scorecards from one parameter file is elite engineering discipline—it already caught real integration failures (elbow belt plane off by 9.4 mm, missing torque pins, kiwi base instability) before spending a dollar.
2. **Torque-only cartridge load-path architecture (D3) + V-band arm-detach split (D1).**
   Recognizing early that printed faces cannot carry arm bending (`14–22 N·m`) and separating cartridges (torque-only via radial torque pin + bone-mounted bearing pairs) from the bending-loaded arm detach (off-the-shelf 3" stainless quick-release sanitary V-band clamp, $25 each) is mechanically sound once the wrist/head exceptions and shaft assembly bugs are fixed.
3. **Whole-workspace actuator sizing (`docs/rnd/spring.md`) + carbon-tube bone stiffening (D4d).**
   Checking 6,000 random 3D poses caught the non-obvious upper-arm yaw overload (`DM-J4310` failed at 0.65× → upgraded to `GIM6010-8 × 2` at 1.84×) that planar straight-arm checks miss, and using Ø40/Ø30 mm roll-wrapped CF tubes solves the dominant stiffness bottleneck of FDM robot links at low mass (+~60 g) and low cost ($70).
4. **Fixed head on top of the mast (D8) with independent ESP32-S3 CAN face/touch node (D11).**
   Keeping the head fixed at 1.29 m maintains a consistent high camera vantage, sheds 0.94 kg from the moving lift carriage, and eliminates the mast poking behind the head; isolating the touch UI on its own CAN node ensures local status/commands survive a Pi 5 hang.
5. **Staged physical risk-retirement plan (`docs/COUPONS.md` → M1 single-arm bench test → full robot).**
   Designing ~$25 / 7.5 h targeted test coupons (C1–C9) to measure the exact FDM unknowns simulation cannot resolve (clearances, press fits, KX slip/creep, HTD tooth mesh, motor bolt circles) before purchasing $1,617 of actuators is the right hardware development strategy—and will immediately expose Findings #2, #3, #8, and #16 cheaply.