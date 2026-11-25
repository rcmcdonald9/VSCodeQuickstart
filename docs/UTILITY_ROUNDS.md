# What should the robot actually do? Four rounds of research

Goal: real, **hands-on** value (not just a camera on legs), at a sane cost, without turning you into a full-time assembler.

Your candidate jobs: weed whacking, laundry, picking up toys, golf (carrying clubs, tracking and finding balls, with your radar), and an outdoor "nanny cam" for the kids.

---

## Round 1: score every job against precedent

Scores run 1–5. **Feasibility** means at a hobby budget, judged against what already exists.

| Job | Hands-on value | How often | Feasibility | Safety risk | Needs legs? | What precedent says |
|---|---|---|---|---|---|---|
| **Fold laundry** | 5 | 4 | **1** | low | no | The two big attempts died: Laundroid ($16k machine, ~$89M raised, bankrupt in 2019) and FoldiMate (needed pre-oriented clothes). Folding is still a research problem. **Rejected.** |
| **Move laundry** (basket from room to washer) | 3 | 3 | 4 | low | **only for stairs** | Easy on one floor, hard on stairs (payload). |
| **Pick up toys** | 4 | 5 | 3 | low | no | Roborock's Saros Z70 vacuum has an arm. It costs $2,599, is rated for 300 g, and gets mixed reviews ("a proof of concept with a price tag"). Open-source learned picking with SO-101 arms reaches roughly 30–80 % success on simple objects. **Grasping one toy at a time is the hard way.** |
| **Collect golf balls** (backyard or practice field) | 4 | depends on how often you practise | **4** | low | no | Autonomous range pickers cost **€23k–$50k+** (ECHO, Korechi). The mechanism inside is a dumb rolling picker that costs $100–300 as a push tool. The value is real and the mechanism is cheap; only the autonomy is expensive. |
| **Carry clubs** (follow-me caddie) | 3 | per round | 4 | low | no | A mature market: follow-me trolleys run **$1,200–4,000** (Stewart, Navee, Axglo, Robera, HelloCaddy). |
| **Find balls / track shots** (radar) | 3 | per session | depends on the radar | low | no | Launch monitors like the Garmin R10 export shot data through a partner API or JSON converters. |
| **Weed whacking / edging** | 4 | weekly in season | 3 | **high** | no | Robot-mower makers now sell trimmer modules (Yarbo, Ecovacs, Anthbot, Segway Navimow). Reviews: useful, fussy to map. |
| **Outdoor nanny cam** | 2–3 | often | 4 | low | no | Mostly software (person following). Low hands-on value. |
| **Dog-poop pickup** (if you have a dog) | 4 | daily | 2–3 | low | no | Beetl announced a poop-scooping yard robot in 2019. I found no sign it ever shipped, so the niche is still unsolved. |
| **Yard cleanup** (sticks, nuts, pine cones, tennis balls) | 3 | weekly in season | **4** | low | no | Rolling nut and ball pickers cost **$30–60**. It's the same mechanism as golf. |

**Round 1 takeaways**

1. **None of your jobs needs legs**, except moving laundry upstairs. Legs cost about $600–800 in motors, a lot of complexity and most of the battery.
2. The jobs with the most value per dollar share one pattern: **move around a yard or floor and collect stuff**, whether golf balls, toys, nuts or sticks. Precedent says to collect with a **dumb mechanism** (a roller picker, a scoop) rather than a clever hand.
3. **Laundry folding is a trap.** Skip it.

---

## Round 2: what form fits those jobs?

| | **A. Dog v3** (8 motors, legged) | **B. Wheeled base + robot arm** (LeKiwi / XLeRobot style) | **C. Mini "skid-steer" tool carrier** |
|---|---|---|---|
| Precedent | Doggo, Minitaur, Pupper | XLeRobot: two SO-101 arms on a wheeled base for **~$660**, running on a Pi 5 | Skid-steer loaders with **universal quick-attach plates** (SAE J2513), the tractor 3-point hitch, Yarbo's modular yard robot, robot mowers with trimmer modules |
| Toys | One at a time with a mouth gripper; slow | Learned grasping, 30–80 % success | **Scoop + rake bucket:** sweeps several at once, no learning needed |
| Golf / yard balls | Poor (no payload) | Poor (one at a time) | **Tows a $30–300 roller picker:** hundreds of balls per run |
| Carry clubs / laundry | No (~1 kg payload) | Light loads | **Yes: 10–20 kg** on a rack or hitch |
| Weed edging | No | Risky | Trimmer module on the plate (later) |
| Grass | OK | Small wheels struggle | Hoverboard hub motors handle short lawns (bigger 8.5–10" wheels for rough grass) |
| Stairs | Yes, eventually | No | No |
| Parts cost | $1,050–1,350 | ~$660–1,200 | **~$350–550 + tools** |
| Hands-on build | ~1.5 h | several hours (arms) | **~1–2 h** |
| Personality | Highest | Medium | Give it the dog's head on a short mast and it becomes a "working dog" |

**Round 2 takeaway:** **C wins on value per dollar for your list.** B's arm becomes an *optional tool* for C later, rather than a separate robot.

---

## Round 3: push on cost and assembly

### Cost: harvest mass-produced subsystems instead of building them

| Subsystem | Harvest from | Why | ~Cost |
|---|---|---|---|
| **Drive motors, motor controller, 36 V battery with BMS, charger** | **A used hoverboard** | Hackers have reflashed hoverboards for years with open-source motor-control firmware (EFeru's hoverboard-firmware-hack-FOC: speed and torque control, serial from a computer). Hackaday's summary: high torque, direct drive, controller, battery and charger, all in one cheap package. | **$30–100** |
| **Ball / nut / pine-cone collector** | A **rolling nut and ball picker** ($30–60) or a **push range picker** ($100–300) | The robot just tows it. That's what the $23k–50k range robots do with their picker gangs. | $30–300 |
| **Bucket, rake, hitch, tool plate** | Printed on your P2S + 1–2 cheap linear actuators | Skid-steer geometry is simple and strong | $60–100 |
| **Laundry or golf-bag rack** | Any laundry basket or bag strap + printed clamp | | $15 |
| **Follow-me** (caddie, nanny cam) | Camera person-tracking (already in our software) + optional UWB tag ($20–40) | How follow-me trolleys work | $0–40 |
| **Brain, voice, camera, Claude** | **Everything we already built** | Same Pi 5 software stack | $0 |

### Assembly: make the tools dumb and the robot smart

1. **One universal tool plate.** A small version of the skid-steer quick-attach: a lip on top, two locking pins below. Every tool (bucket, picker hitch, basket rack, camera mast, trimmer, the arm later) has the same plate.
2. **The robot changes its own tools.** It drives up to a **tool rack** (a printed stand in the garage), tilts the plate under a tool's lip, lifts, and locks. That's how real skid-steers couple, and how CNC tool changers work. "Go pick up the golf balls" doesn't need you to bolt anything on.
3. **The hoverboard is already assembled.** Keep its motors, board and battery in their original frame. A printed spine clamps the two halves rigid. Your hands-on work is about 20 screws, a firmware flash, and a few plugs.
4. **No custom wiring harness.** Each tool has one plug, a blind-mate connector in the plate, so the robot can also power tools.

### Using your radar

- **If it's a golf launch monitor** (Garmin R10, Mevo+, TrackMan-style): each shot's carry distance and direction gives a **predicted landing spot**. The robot drives there to confirm with its camera, logs it, and after the bucket it sweeps only the landing zone. Garmin R10 data is available through Garmin's partner API, or from session JSON files using community converters.
- **If it's a mmWave sensor module:** use it for **people and pet detection**. That's ideal as a *safety interlock* (stop the tool when a person or pet is within X m, even in the dark or behind the camera) and for "follow the kid" in nanny-cam mode.

---

## Round 4: risks and a concrete plan

**Risks and mitigations**

| Risk | Mitigation |
|---|---|
| Bucket misses toys against walls or under furniture | A rake arm sweeps toys out first. Anything left gets a photo and a "these 3 need a human" message. Aim for ~80 % automatic. |
| Small hoverboard wheels in wet or long grass | Fit 8.5–10" off-road wheels, or use two hoverboards (four motors, true skid-steer) |
| A string trimmer near kids or pets | **Last** to be added. Low-inertia nylon blades, radar and camera person stop, lift/tilt stop, dead-man mode until it's proven |
| Hoverboard firmware flash | The community-documented process needs a $5 ST-Link. Done once, by me with you over a call, or buy a board that's known to be flashable. |
| It's not a dog anymore | It keeps Pip's head (screen eyes, camera, voice, Claude) on a short neck. The legged v3 stays a separate companion project that shares head, brain and software. |

**Plan (each stage is useful on its own)**

| Stage | Build | Cost | Useful for |
|---|---|---|---|
| 1 | Hoverboard base + Pip head on a mast + bucket/rake | ~$250–400 | **Picking up toys** (indoor), nanny cam, voice |
| 2 | Tool plate + tool rack + golf/nut roller picker | +$60–300 | **Golf balls / yard cleanup** with automatic tool change; radar integration |
| 3 | Rack for laundry basket or golf bag + follow-me | +$40 | **Caddie / laundry transport** (one floor) |
| 4 (optional) | SO-101 arm as a tool | +$120–230 | Grasping odd items (learned) |
| 5 (later, careful) | Trimmer module | +$100–200 | Edging |

---

## Sources

- [Roborock Saros Z70 review (Gizmodo)](https://gizmodo.com/roborock-saros-z70-review-this-robovacs-robotic-arm-is-a-swing-and-a-miss-2000607909)
- [Roborock Saros Z70 (BGR)](https://bgr.com/reviews/roborock-saros-z70-review-do-robot-vacuums-need-arms/)
- [Roborock Saros Z70 (TechRadar)](https://www.techradar.com/home/robot-vacuums/3-things-i-loved-about-the-worlds-first-robovac-with-a-mechanical-arm-and-3-i-didnt)
- [XLeRobot (GitHub)](https://github.com/Vector-Wangel/XLeRobot)
- [XLeRobot docs](https://xlerobot.readthedocs.io/en/latest/)
- [XLeRobot paper (arXiv 2603.09051)](https://arxiv.org/pdf/2603.09051)
- [SO-101 pick-and-place case study (OpenELAB)](https://openelab.io/blogs/learn/so-arm101-pick-and-place-lerobot-act-case-study)
- [Wizwand SO-101 benchmarks](https://www.wizwand.com/sota/stack-blocks-on-lerobot-so-101-real-world-test)
- [LeKiwi (Seeed wiki)](https://wiki.seeedstudio.com/lerobot_lekiwi/)
- [IEEE Spectrum: future of laundry-folding robots](https://spectrum.ieee.org/is-there-a-future-for-laundry-folding-robots)
- [The Robot Report: Laundroid bankruptcy](https://www.therobotreport.com/consumer-robots-still-struggle-as-laundroid-maker-seven-dreamers-files-for-bankruptcy/)
- [Alf Rehn: requiem for Laundroid](https://www.alfrehn.com/blog/requiem-for-an-innovation-what-we-can-learn-from-the-sad-story-of-laundroid-the-laundry-folding-robot)
- [FoldiMate (Wikipedia)](https://Www.wikipedia.org/wiki/FoldiMate)
- [ECHO robotic golf ball collector](https://echorobotics.com/en-au/garden-robot-solutions/robotic-golf-ball-collector)
- [Golf Monthly: $50k+ robot ball collector](https://www.golfmonthly.com/news/the-usd50-000-robot-ball-collector-spotted-on-the-us-open-driving-range-at-oakmont)
- [The Robot Report: Korechi Pik'r](https://www.therobotreport.com/korechi-innovations-pikr-1500-robot-retrieves-golf-balls-at-the-u-s-open/)
- [Golf.com: robotic range pickers](https://golf.com/gear/robotic-driving-range-pickers-have-arrived/?amp=1)
- [Push golf ball pickers (Walmart)](https://www.walmart.com/c/kp/range-ball-picker)
- [Rolling golf ball picker (ParWest)](https://parwest.com/product/rolling-golf-ball-picker-with-retractable-handle/)
- [Navee Birdie 3X review (Golfalot)](https://golfalot.com/equipment-review/navee-birdie-3x-electric-trolley-review)
- [Robera Pro follow caddy](https://reach.dog/shop/thegolftrainingaidstorecom/products/robera-pro-remote-follow-ai-electric-golf-caddy)
- [HelloCaddy](https://deals.blavity.com/sales/hellocaddy-smart-caddy-robot)
- [Axglo E5 follow-me trolley](https://www.americangolf.co.uk/golf-trolleys/axglo-e5-follow-me-extended-range-electric-golf-trolley-package-469788.html)
- [Garmin and Noonan: Golf Premium API](https://seekingalpha.com/pr/20240406-garmin-and-noonan-collaborate-to-bring-launch-monitor-data-seamlessly-onto-the-course)
- [Garmin forums: R10 data export](https://forums.garmin.com/outdoor-recreation/golf/f/approach-r10/295014/r10-data-export)
- [SlashGear: Yarbo trimmer module](https://www.slashgear.com/1948382/yarbo-mower-pro-lawn-edger-trimmer-module-revealed)
- [Ecovacs robot mower with string trimmer (Galaxus)](https://www.galaxus.ch/en/page/robot-lawnmower-with-string-trimmer-ecovacs-solves-the-edge-problem-41544)
- [Techlicious: Yarbo autonomous trimmer](https://www.techlicious.com/blog/yarbo-adds-fully-autonomous-trimmer-to-its-modular-lawn-care-robot/)
- [Beetl poop-scooping robot (iHeartDogs)](https://iheartdogs.com/beetl-the-backyard-robot-is-designed-to-pick-up-all-your-dogs-poop)
- [Beetl (CB Insights)](https://www.cbinsights.com/company/beetl)
- [hoverboard-firmware-hack-FOC](https://github.com/alex-makarov/hoverboard-firmware-hack-FOC)
- [EFeru on GitHub](https://github.com:443/EFeru)
- [Hackaday.io: reprogramming hoverboards for robots](https://hackaday.io/project/158256/log/146067-reprogramming-hoverboards-for-robots)
