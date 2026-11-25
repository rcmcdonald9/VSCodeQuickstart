# Pip v2: designed to be assembled by cheap machines

> **Status: implemented** in `cad/` (v2) and verified in simulation ([SIMULATION.md](SIMULATION.md)). Where the build differs from this plan:
> - **The legs are held in by the deck, not a latch.** The deck screws down over the four hip blocks (rule 3, "held by the next part"), so swapping a leg means removing the deck's 4 screws.
> - **Fasteners: M3 × 8 only, self-tapping into printed holes.** No nut traps were needed.
> - **Head:** a pan/tilt neck (2 more STS3215s). The tilt servo slides down a sleeve, and the head drops onto a keyed hub plus an idler pin. Every step is still straight down.
> - **Grip lugs** (flats 20 mm apart, 3 mm datum notch) are on the tray, deck, hip blocks, links, pan bracket and head. The caps and deck are suction-friendly. The feet and the tilt hub are exceptions.
> - **Cost** ended up at about $410–480: the 12 bus servos (2 spares) cost more than the estimate in §6.
> - **Simulation findings:** the turning and drift settings added to the gait are explained in [SIMULATION.md](SIMULATION.md).

**Goal:** Make easy, low-precision assembly the core design constraint, so a person with jigs, a converted old 3D printer, or a ~$150 hobby arm can build most of the robot. Pip v1 (the current CAD) was designed for a human with a screwdriver. v2 starts over from how the robot gets assembled.

## 1. Precedent: what has already worked

| Precedent | What it shows | What we take |
|---|---|---|
| **MIT Center for Bits and Atoms: discrete lattice assembly / "relative robots"** (BILL-E and successors) | They design the robot and the material together. Passive alignment features in the parts correct for an imprecise assembler, so very simple robots can build precise structures. | **Put the precision in the parts, not the assembler.** Every joint gets lead-ins and self-locating features, so a sloppy, cheap machine still lands parts correctly. |
| **Industrial design for assembly (DFA)** (Boothroyd–Dewhurst) | Fewer parts, one assembly direction, symmetric or keyed parts, no loose fasteners, parts held in by the next part. These are the standard levers for automated assembly. | The rules in §3. |
| **Petoi Bittle** (commercial robot dog) | Glue-free, snap-together frame. Built by hand in about an hour. 9 servos (2 per leg + head). The servos still use screws, and the leg calibration is a reported pain point. | Snap-fit frames work for a small dog. Fewer degrees of freedom (8, not 12) makes assembly much simpler. Calibration should be automatic. |
| **SO-100 / SO-101 arm** (Hugging Face LeRobot) | An open-source 6-axis printed arm. About $120–130 in non-printed parts per follower arm, using **Feetech STS3215 bus servos**. It does pick-and-place via imitation learning. | A cheap assembly helper **that shares the dog's servo**, so spares and the software stack overlap. |
| **Print-farm auto-eject** (Looprint, AutoClear, 3DQue AutoFarm3D; the latter's kit is about $129) | Bambu P/X-series printers can push finished parts off a cooled plate with the toolhead, with the door open. | The **P2S itself is the first step of the line**: print, eject into a bin. |
| **Pause-at-layer embedding** (Markforged, Prusa and Bambu slicers) | Nuts and magnets can be dropped into pockets mid-print and get sealed in. Placement is manual. No consumer printer places them automatically. | No heat-set inserts. The P2S traps the hardware during the print. |
| **Gantry pick-and-place machines** (the electronics industry) | Most automated assembly is 3-axis Cartesian plus a nozzle or gripper, not an arm. | A converted cheap 3D printer can serve as the assembler, **if** every step is a straight move down. |

**What precedent says won't work cheaply:** a robot plugging in floppy cables, a robot doing multi-axis insertions, and a robot reaching precision without alignment features. The design has to avoid all three.

## 2. The core architecture

### 2a. Bus servos instead of PWM servos (the biggest single change)
Switch from DS3225 + PCA9685 to **Feetech STS3215, 12 V / 30 kg·cm variant** (45.2 × 24.7 × 35 mm). These are serial-bus servos with a magnetic encoder.

- **Daisy-chained:** about 4 short cables per leg instead of 12 long home-run cables. There's no PCA9685 and no servo power distribution.
- **Runs straight off the 3S LiPo** (rated 4–14 V). The 20 A buck converter and the Wago nuts are gone.
- **Encoder feedback means no mechanical calibration.** The horn can sit on any spline tooth. The software reads each joint's position while the leg is held in a jig pose and stores that as zero. The fussy "find the right tooth" step disappears.
- **Assembly self-test for free.** After each step, ping the bus: is servo #7 present, and does it read the expected angle in the jig? A missed or wrong step is detected right away.
- **Same servo as the SO-101 arm,** which makes the arm the obvious cheap assembly helper.
- ⚠️ STS3215 is sold as both a 7.4 V / 19 kg version and a 12 V / 30 kg version under the same name. **Buy the 12 V version.**

### 2b. 8 degrees of freedom: drop the hip-roll joints
With only hip-pitch and knee per leg, **every joint axis on the robot is parallel**. Each leg becomes a flat stack built up along one axis. The roll joint was the one thing forcing a second assembly direction and a rotating connection at the leg dock.

- **You lose:** sideways stepping and some lateral balance. It still turns by stepping each side differently, like Bittle, and walks fine on flat floors.
- **You gain:** 4 fewer servos (about $80 saved), no left/right part variants, a single-axis build, and simpler kinematics and gait.
- Roll can come back later as a v3 "hip module" option once the line works.

### 2c. Four identical leg modules, no mirrored parts
- Each leg module = **pitch servo + upper link + knee servo + lower link + foot**. That is 2 servos and 3 printed parts.
- All servo long axes lie in the module's vertical plane, so the module is fore/aft symmetric. **The same module fits all four corners,** rotated 180° for the opposite side. Knee direction is set in software (the servos turn a full 360°).
- **Single build axis:** jig → base link → pitch servo (shaft up) → upper link onto the horn → knee servo → lower link onto the horn → foot. Each part goes **straight down**.

### 2d. The body is a docking tray with one wired part
- The body **tray** sits in a fixture. The four leg modules **drop straight down into keyed corner sockets** and lock with a printed snap latch. They come out again by pressing the latch, with no tools.
- **Blind-mate electrical dock:** each socket has a magnetic pogo-pin connector (2 power pins doubled for current, 1 data, ground). The leg's connection is made the moment it drops in. There is no rotating joint at the dock (no roll), so the connector is fixed.
- All wiring lives in one place:
  - **Phase 1:** a hand-made harness. The bus adapter, the 4 dock connectors and a 5 V buck for the Pi sit on a printed carrier that drops into the tray.
  - **Phase 2:** a small custom **backplane PCB** (cheap fab, about $20–40 for a few boards). It carries the bus driver, the 5 V supply, the 4 dock pads and the Pi connection. The Pi and the battery plug into it. That's the "spine" every other part docks to.
- After that, everything stacks down: battery tray → backplane → Pi deck → head. Each layer is trapped by the one above it.

## 3. Design rules (every v2 part must follow these)

1. **One assembly direction per subassembly** (down). At most one reorientation per module.
2. **Precision in the part.** Every mating feature has a lead-in chamfer or cone with at least **±1.5 mm capture range**, plus a key so the wrong orientation can't fit. A gantry or hobby arm can be 1 mm off and still succeed.
3. **Held by the next part.** A part is captured by the part above it. Fasteners only clamp layers together.
4. **One fastener type, always vertical, always captive.** For example, one size of thread-forming screw for plastic, pre-seated in printed collars so the tool only has to push and spin. No heat-set inserts. Where a nut is needed, the P2S traps it at a pause-at-layer.
5. **Identical modules, no left/right variants.**
6. **No free wires during assembly.** Bus servos plus blind-mate docks. The short servo-to-servo chain cables are the one exception (see §5).
7. **A standard handling feature on every printed part:**
   - two parallel grip flats, **20 mm apart**
   - a **3 mm datum notch** so the part's position is known
   - a flat, unbroken **suction pad of at least 20 mm diameter** on the top face

   One gripper, or one suction cup, then handles every part.
8. **Printed in assembly orientation** wherever possible, so parts come off the plate already right-side-up for the kit tray.
9. **Every step is verifiable**, by servo readback or by a jig with a go/no-go feature.
10. **Serviceable:** a leg module swaps in about 10 seconds with no tools.

## 4. The cheap assembly line, cheapest first

| Tier | Tool | ~Cost | Does |
|---|---|---|---|
| 0 | **You + printed jigs + kit trays + electric screwdriver** | $30–60 | Everything. Design for this first: anything easy for a person with jigs is possible for a machine. |
| 1 | **The P2S itself** | $0 | Prints parts in assembly orientation. Traps nuts and magnets at a pause (you drop them in). Pushes finished parts off into a bin with the door open. |
| 2 | **Gantry assembler** from a used bed-slinger or Cartesian printer, with the hotend swapped for a tool head (suction cup + small vacuum pump, servo gripper, and a sprung screwdriver bit) | $100–250 | Straight-down place, press, snap and screw steps from kit trays. A printer gantry's ~0.1 mm repeatability is far better than the ±1.5 mm the parts tolerate. |
| 3 | **SO-101 arm** (same STS3215 servos) | ~$120–230 in parts | The steps a gantry can't do: reorienting a subassembly between jigs, moving parts from the eject bin into kit trays, dropping modules into the body. It is imprecise (mm-level), but the lead-ins cover that. |
| ✗ | $1.5k+ industrial or desktop arm | | Not needed once the design follows §3 |

## 5. Still done by hand (honest list)

- **Plugging servo chain cables:** about 2 per leg module, 8 total. Use pre-made cables. A robot plugging floppy connectors isn't realistic at this budget.
- **Fitting the horn screw on each servo,** unless the servos arrive with horns fitted.
- **Loading kit trays.** About a minute per tray, unless the SO-101 does it.
- **Dropping nuts in at print pauses.** About 30 seconds per plate.
- **Battery connection and charging** (safety).

**Target:** about 20–30 minutes of human work per dog. The machines do roughly 70–80 % of the mechanical steps. This is a design target, not something we've measured yet.

## 6. What changes from v1

| | v1 (current) | v2 (assembly-first) |
|---|---|---|
| DoF | 12 | 8 |
| Servos | 12 × DS3225 PWM | 8 × STS3215 bus (12 V) |
| Servo control | PCA9685 + 12 cables + Wago power | Bus adapter / backplane + daisy chain |
| Power | LiPo → 20 A buck → servos; 5 V buck → Pi | LiPo → servos directly; 5 V buck → Pi |
| Leg variants | Normal + mirrored hip housings | One module type, ×4 |
| Calibration | Manual (spline tooth + trim, per joint) | Automatic: read encoders in a jig pose |
| Inserts | 56 heat-set inserts | None (pause-at-layer nuts + thread-forming screws) |
| Assembly directions | Many (slide-in servos, sideways bolts) | Straight down per subassembly |
| Rough parts cost (no Pi) | $330–420 | About $300–380. The servos cost more each but there are fewer, and the PCA9685 and big buck go away. |

**Software impact:** small. `servos.py` gets a bus-servo backend. The `config.yaml` geometry and leg model become 2-DOF, so the IK simplifies and the gait drops roll. `calibrate.py` becomes "put the leg in the jig, press enter". Vision, navigation, voice and the Claude brain are unchanged.

## 7. Build order for v2

1. **One leg module + its jig.** Prove the single-axis stack, the lead-ins, the grip flats and the encoder auto-calibration by hand (tier 0).
2. **Body tray + dock sockets + phase-1 harness.** Prove that legs drop in and lock and that the pogo connection is reliable while walking.
3. **Kit trays + gantry tool head** (tier 2). Automate the leg-module stack.
4. **SO-101** (tier 3) for transfers and docking.
5. **Backplane PCB** (phase 2) to remove the last harness.

## Sources
- [MIT CBA: Smith, Abdel-Rahman, Gershenfeld, ICRA 2024: relative robots for discrete lattice assembly](https://cba.mit.edu/docs/papers/24.05.ICRA.pdf)
- [MIT News: robots that build large structures from little pieces](https://news.mit.edu/2019/robots-large-structures-little-pieces-1016)
- [Jenett, PhD thesis 2020: discrete lattice assembly](https://cba.mit.edu/docs/theses/20.09.jenett.pdf)
- [Petoi Bittle frame assembly guide](https://bittle.petoi.com/3-assemble-the-frame)
- [Tom's Hardware: Petoi Bittle review](https://www.tomshardware.com/reviews/petio-bittle-opensource-robotic-dog-ai-stem-learning)
- [LeRobot SO-101 documentation](https://huggingface.co/docs/lerobot/so101)
- [Hackster: SO-101 launch (non-printed parts about $130)](https://www.hackster.io/news/hugging-face-launches-the-so-101-an-upgraded-low-cost-3d-printable-autonomous-robot-arm-532360f441eb)
- [RobotShop: Feetech STS3215 12 V 30 kg·cm](https://robotshop.com/products/feetech-12v-30kgcm-magnetic-encoding-servo-sts3215)
- [Waveshare ST3215 servo](https://www.waveshare.com/st3215-servo.htm?sku=22414)
- [Core Electronics: STS3215 (7.4 V variant listing)](https://core-electronics.com.au/feetech-sts3215-smart-servo.html.md)
- [Tom's Hardware: $129 auto-ejection kit for Bambu print farms](https://tomshardware.com/3d-printing/new-auto-ejection-tool-for-bambu-lab-print-farms-automatically-ejects-finished-3d-prints-from-the-machine-usd129-kit-includes-auto-door-opener-and-special-bed-surface-for-frictionless-part-ejection)
- [Looprint: open-source repeat printing with push-off](https://nickiandersen.github.io/looprint)
- [Bambu forum: project to automatically remove parts from the build plate](https://forum.bambulab.com/t/project-to-automatically-remove-parts-from-build-plate/17231?page=4)
- [Markforged: embedding nuts in 3D prints](https://markforged.com/blog/embedding-nuts-3d-printing)
