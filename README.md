# Pip: a 3D-printed AI robot dog

A small (about 21 cm hip-to-hip, ~1.5 kg) four-legged robot dog that you print on a **Bambu Lab P2S** and run on a **Raspberry Pi 5**. It walks with a trot gait and sees with a Pi camera plus an on-board object detector. It listens for voice commands offline, talks back, and can hand harder requests to **Claude**, which plans multi-step tasks using the dog's tools and camera.

![Pip assembly render](docs/img/assembly.png)

> "Hey Pip, go find the ball." → turns until it sees the ball → walks over → *"Found it! It's right here by the couch."*

## What's in this repo

| Path | What |
|------|------|
| [`cad/`](cad) | Parametric OpenSCAD design. [`cad/stl/`](cad/stl) has ready-to-print STLs in print orientation, no supports needed. |
| [`software/`](software) | Python control stack: kinematics, gait, vision, navigation, voice, AI brain, calibration tool. 57 tests, and it runs in simulation on any computer. |
| [`docs/BOM.md`](docs/BOM.md) | Shopping list (about $330–420 on top of the Pi) |
| [`docs/PRINTING.md`](docs/PRINTING.md) | P2S slicer settings, plates, test-fit procedure |
| [`docs/WIRING.md`](docs/WIRING.md) | Power and GPIO wiring, servo channel map |
| [`docs/ASSEMBLY.md`](docs/ASSEMBLY.md) | Step-by-step build and calibration |
| [`docs/SOFTWARE.md`](docs/SOFTWARE.md) | Architecture, running it, setting up Claude, tuning |

## The plan, end to end

1. **Order parts** from [BOM.md](docs/BOM.md). The 12 servos and the LiPo setup are the big items.
2. **While you wait:** run the software in simulation (`cd software && pip install -e .[dev] && python -m robodog --sim --local`), and get an Anthropic API key if you want the AI brain.
3. **When the P2S arrives:** print the **test-fit pair** first ([PRINTING.md §1](docs/PRINTING.md)). Check it against a real servo, then print the rest (about 12–16 h, ~400 g PETG + a little TPU).
4. **Bench-wire the electronics** and run `setup_pi.sh` ([WIRING.md](docs/WIRING.md), [ASSEMBLY.md §0](docs/ASSEMBLY.md)).
5. **Assemble** with the servos centred, then **calibrate** ([ASSEMBLY.md](docs/ASSEMBLY.md)).
6. **First walk** with typed commands, then switch on voice and Claude.

## Specs

| | |
|---|---|
| Degrees of freedom | 12 (3 per leg: roll, pitch, knee) |
| Actuators | 12 × 25 kg·cm standard servos via PCA9685 |
| Leg segments | 100 mm + 100 mm, ~170 mm standing hip height |
| Compute | Raspberry Pi 5 (8 GB), optional AI HAT+ |
| Sensors | Camera Module 3, VL53L1X time-of-flight, optional IMU |
| Voice | Vosk (offline STT) + Piper (offline TTS), wake word "Hey Pip" |
| AI | Claude via the Anthropic API (tool use + vision). Offline keyword fallback. |
| Power | 3S 2200 mAh LiPo → 6.8 V servo rail + 5 V Pi rail |

## Roadmap ideas

- [ ] Hailo AI HAT+ detector backend (much faster vision)
- [ ] IMU-based balance correction while walking
- [ ] Battery voltage monitoring (ADS1115) so Pip can say when it's hungry
- [ ] Tail servo on a spare PCA9685 channel
- [ ] Bearing-supported idle side on the knee joints (v2 legs)

Licensed MIT. Have fun, and keep a hand near it for the first walk.
