# Pip v3 concept: taller, nicer, actually useful, without more assembly work

**Status:** proposal for discussion. Nothing in `cad/` or `software/` has changed yet.

## The problem, restated

You want a dog that:

- **looks good:** slim legs, no bricks
- **walks well:** real swing, faster, stable
- stands **12–14 in (30–36 cm)** tall
- **does something useful** every day
- costs a sane amount
- doesn't turn you into a full-time assembler or printer operator

## The one idea everything else follows from

> **Concentrate all the precision, cost and wiring in a few identical "smart" modules. Make everything else dumb, printed, and snap-together.**

Pip v2 spread complexity everywhere: 10 servos, 10 caps, 88 screws, and cables across joints. v3 puts it in **four identical hip modules**. Everything else is a printed shape that clips on.

## 1. Fewer motors, all of them at the hips (this fixes looks, swing and torque)

**Conventional wisdom says a good quadruped needs 12 motors (3 per leg). Precedent says 8 can be excellent:**

- **Ghost/Penn Minitaur** has two active joints per leg and runs at 1.5–1.9 m/s.
- **Stanford Doggo** has 8 motors, built for under $3,000 from mostly off-the-shelf parts, open source. It drives each leg through belts with **coaxial shafts at the hip**. It matched the best-performing animal at vertical jumping.

**v3 legs:**

- Both motors of each leg sit coaxially at the hip.
- The knee is driven through a printed **pushrod**, so the leg itself has **no motor in it**.
- The leg becomes a thigh and shin of printed links on bearings: thin, light, and dog-shaped.

| What it fixes | Why |
|---|---|
| **Bulky knees** | There's no knee motor anymore |
| **Small, slow swing** | The leg weighs a fraction of v2's, so the same motor swings it much faster |
| **Knee servo at its torque limit** | In v2 the load was mostly from swinging the heavy knee servo (see SIMULATION.md). That mass is gone, and the motors are far stronger. |
| **Cost** | 8 motors instead of 12 |

**Motor:** SteadyWin GIM6010-8, about $70–95 each. It's brushless with an 8:1 gearbox, a built-in driver, CAN bus and a rated 5 N·m. 8 of them cost **about $560–760** (12 would be $850–1,150).

**Rough sizing check** (to be verified in the simulator):

- 5 kg robot at 26 cm hip height, with 15 + 15 cm links.
- In a trot each leg carries about 25 N, putting about 1.8 N·m on the knee standing and about 3.7 N·m at dynamic peaks.
- That's a **1.4–2.8× margin** on the motor's rated torque.

The STS3215 ran at about 0.9× its rating in v2.

**Later, optional:** a home-made actuator (a $20–40 brushless motor, a gearbox printed on your P2S, and a $20–50 driver). Published designs report about $30–40 per joint. Test it on the leg test stand against a commercial motor before trusting it.

## 2. A legs + belly-wheels hybrid (this is where the daily usefulness comes from)

Precedent says legged robots on wheels are the future of home and site robots: Unitree Go2-W and ANYmal on wheels are commercial products. But wheels on the feet add motors and cables at the far end of every leg, exactly the weight we just removed.

**The cheaper twist:** put **two small drive wheels plus two ball casters in the belly**, like a robot vacuum.

- To **roll**, the dog lies down onto its belly wheels and drives like a vacuum: quiet, efficient, precise.
- It **stands up and walks** for thresholds, rugs, cords, toys, stairs, or just to be a dog.

| Benefit | Why it matters |
|---|---|
| **Patrol all day** | Rolling uses a fraction of the energy of walking |
| **Quiet at night** | No foot-stomping |
| **Self-docking** | It rolls onto a charging dock as precisely as a vacuum does: belly contacts, no human |
| **Turning is solved** | The 8-motor legs' weak point (skid turns) mostly disappears, because it turns on its wheels |
| **Cost** | 2 small geared motors with encoders + 1 driver, about $25–40. No cables cross a leg joint. |

**Honesty check:** I haven't found a robot dog that does exactly "lie down and roll". The two halves, vacuum-style drives and wheeled quadrupeds, are both proven. This is the most novel idea here, and the simulator can test it before you buy anything.

## 3. Walking learned in the simulator, not hand-tuned

v2's gait was hand-written and tuned in the sim. **v3 would use a walking controller trained by reinforcement learning (RL) in MuJoCo**, the same simulator we already use. The training loop is MJX/PPO, running many copies of the robot at once on a GPU.

- **Precedent:** this is how Stanford's Pupper v3 is taught to walk. Their course trains PPO policies in MJX and deploys them on the real robot. A student project ran both a locomotion policy and a vision model on a Pi 4 and Pi 5.
- **Brushless motors make the jump from sim to the real robot much easier** than hobby servos: they can be torque-controlled and they're easier to model.
- **The result:** a natural, robust gait with big swing, push recovery, and the stand / sit / lie-down-to-roll transitions.
- **Training needs a GPU** for a few hours. A rented cloud GPU costs a few dollars per run.

## 4. The build: making it easy for you, not for a robot arm

| Area | v2 | v3 plan |
|---|---|---|
| Structure | One 8.5 h printed tray | **Two carbon or aluminium tubes as the spine.** Body length is just tube length, so changing proportions is a saw cut, not a reprint. Hip modules clamp on with printed **cam clamps** (no tools). |
| Motor mounting | 10 cups + caps, 4 screws each | **Clamshell hip module:** 2 motors drop into one half, the other half closes, **4 screws per module** |
| Leg joints | Horn screws | Bearings **dropped into pockets during a print pause** (a documented technique, e.g. Disney Research's embedded bearings), so the printer captures them. Joints are closed with **push-in snap pins**, with a pressed steel dowel as the fallback. |
| Wiring | 10 servo IDs, 8 chain cables, pogo docks | **One cable per hip module** (CAN + power) to a central hub: **4 plugs**. The two motors inside each module come pre-chained. |
| Look | Bare structure | **Shells held on by magnets** dropped in at print pause. They swap in seconds, and you can restyle the dog without touching the mechanism. |
| Feet | TPU ball + screw | TPU foot that snaps on |
| Head | Pan/tilt sleeve, hub, latch | Same 2 STS3215 servos, plus **two round LCD "eyes"** (about $5 each, plug-in). Expressive screen eyes are what gives EMO / Vector / Loona-style robots their personality, for almost no assembly. |

**Manual work budget (target):**

| | v2 | v3 target |
|---|---|---|
| Screws | 88 | ~25 |
| Cables or connectors you plug | ~12 + 10 servo-ID steps | ~8 |
| Inserts | 0 | 0 |
| Print-pause drop-ins (bearings, magnets) | 0 | ~30 items at ~5 s each |
| Hands-on build time | ~3–4 h | **~1.5 h** |

**CAD tool change:** the "brick" look is partly OpenSCAD's fault, because it can't fillet edges or blend shapes. I'd move v3 to **build123d**, a Python CAD library with fillets and smooth blended shapes that exports STEP and STL. It's still scripted and simulator-linked, but it can produce rounded, organic shapes.

## 5. Size and specs (target)

| | |
|---|---|
| Height | ~33 cm (13 in) to the top of the head, ~26 cm hip height |
| Length | ~42 cm body, ~50 cm with head |
| Weight | ~4.5–5.5 kg |
| Motors | 8 × GIM6010-8 (legs) + 2 × STS3215 (head) + 2 small gearmotors (belly drive) |
| Battery | 6S Li-ion with BMS (~22 V), charged through belly contacts on the dock |
| Sensing | Camera Module 3, small 2D lidar for mapping and docking (~$70–100), ToF |
| Compute | Pi 5 (yours); optional AI HAT+ later |
| Parts cost (rough) | **~$1,050–1,350** (motors $560–760; battery and charger ~$130; lidar ~$80; frame, bearings, tubes ~$80; electronics, eyes, dock ~$120; filament ~$50) |

## 6. What it does for you (daily utility)

1. **Patrol and check-in** (the main use):
   - **Rolls** its route quietly. **Stands up** to look over things or step over thresholds.
   - Photographs the stove, doors, windows, under the sink and the pets.
   - **Claude summarizes:** "Back door open, stove off, cat asleep on the couch."
   - Sends this on a schedule or when you ask from your phone.
2. **"Where did I leave…?"** Every patrol logs what it saw and where (lidar map + vision), so it can answer "where are my keys / glasses / remote?" with a photo and a room.
3. **Telepresence:** drive it from your phone, look around, talk through it.
4. **Pet companion:** check on the pets, play with a ball, live view.
5. **Yard scout** (later, needs weather-proofing): walks the yard, shares parts and software with your mower project.

## 7. Ideas I considered and rejected (and why)

- **12 motors, the "standard" layout:** +$300–400 and more assembly. Hip roll mainly helps on rough ground and with sideways steps, and the belly wheels cover turning. Keep it as a later upgrade.
- **Bigger hobby servos at 30+ cm:** slow and stiff, the exact walk you didn't like. Brushless is the real fix.
- **One motor per leg with a walking linkage** (Jansen Strandbeest / Klann style): 2–4 motors total, cheapest by far. But it can't pose, sit or recover, and it looks like a spider, not a dog.
- **Leg joints that bend the plastic** (flexures) instead of using bearings: fewer parts, but likely to fatigue under a 5 kg robot's impacts.
- **Printing directly over the motors:** zero assembly, but a $75 motor gets glued into a $2 print. Reserved for bearings and magnets.
- **A robot assembly line:** at 1–5 robots, a ~1.5 h hand build is far cheaper. Keep the straight-down design rules so automation stays possible later.

## 8. How to iterate fast (and cheaply)

| Stage | What | Cost | Answers |
|---|---|---|---|
| **0. Simulator only** | build123d CAD → MuJoCo; GIM6010-8 motor model; RL-trained gait; test the belly-roll and lie-down transitions | $0 (plus a few $ of cloud GPU) | Does the size and motor choice work? How fast does it walk? Do the poses and transitions work? |
| **1. One-leg test stand** | 1 hip module + 1 leg on a vertical slide | ~$250 | Do the real motor's numbers match the simulator? Do the bearings and pushrod hold up? |
| **2. Full robot** | 4 modules + spine + shells + belly drive | ~$800–1,100 more | Everything |
| **3. Utility software** | Docking, patrol, object memory, phone app | $0 | Daily value |

## Sources

- [Stanford Doggo paper (arXiv 1905.04254)](https://arxiv.org/pdf/1905.04254)
- [Stanford News: Doggo](https://news.stanford.edu/2019/05/20/dog-like-robot-jumps-flips-trots/)
- [TechCrunch: Stanford Doggo](https://techcrunch.com/2019/05/20/stanfords-doggo-is-a-petite-robotic-quadruped-you-can-maybe-build-yourself)
- [Kenneally, De, Koditschek: Design Principles for a Family of Direct-Drive Legged Robots](https://kodlab.seas.upenn.edu/design-principles-for-a-family-of-direct-drive-legged-robots/)
- [Austin et al.: Minitaur gait optimization](https://astronomicaltelescopes.spiedigitallibrary.org/conference-proceedings-of-spie/10195/1019504/Gait-design-and-optimization-for-efficient-running-of-a-direct/10.1117/12.2262898.full)
- [IEEE Spectrum: Ghost Robotics Minitaur](https://spectrum.ieee.org/ghost-robotics-minitaur-demonstrates-impressive-new-skills)
- [Stanford CS123 Lab 5: How to Train Your Dog (MJX PPO, sim-to-real)](https://cs123-stanford.readthedocs.io/en/latest/schedule/labs/fall-24/lab-5.html)
- [Pupper v3 docs: MuJoCo simulation and neural controller](https://pupper-v3-documentation.readthedocs.io/en/latest/development/modifying_code.html)
- [Stanford CS224R project: locomotion + vision on Pi 4/5](https://cs224r.stanford.edu/projects/pdfs/CS224R_Final_Report%20(18).pdf)
- [All3DP: Disney Research embedded ball bearings](https://all3dp.com/embedded-ball-bearings-balance-3d-prints/)
- [Hackaday: the pause-and-attach technique](https://hackaday.com/2024/10/05/see-the-pause-and-attach-technique-for-3d-printing-in-action)
- [SteadyWin GIM6010-8 (OpenELAB)](https://openelab.io/a/s/products/steadywin-gim6010-8-robot-motor)
- [Hackster: 3D-printable QDD actuators (~$40)](https://www.hackster.io/news/caden-kraft-s-3d-printable-quasi-direct-drive-actuators-cost-as-little-as-40-each-b1f5fa7376d7)
- [CircuitDigest: robot dog with 12 homemade actuators](https://circuitdigest.com/news/robot-dog-gets-12-homemade-actuators-each-with-a-custom-foc-driver-and-3d-printed-gearbox)
