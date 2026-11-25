# Parts list (v2)

Prices are rough USD estimates. You already have the **Raspberry Pi 5 (8 GB)** and the **Bambu Lab P2S**.

> **Changes from v1:** smart bus servos instead of PWM servos, so the PCA9685 board, the 20 A buck converter, the Wago nuts and all 56 heat-set inserts are gone. There's one screw size for everything. The servo count is 10 (8 for the legs, 2 for the head).

## Must-have

| # | Part | Qty | ~Price | Notes |
|---|------|-----|--------|-------|
| 1 | **Feetech STS3215 serial-bus servo, 12 V / 30 kg·cm version** (Waveshare sells it as "ST3215") | **10 + 2 spares** | $200–265 | ⚠️ It's also sold as a 7.4 V / 19 kg version under the same name. **Buy the 12 V one.** Buy all from one seller. Same servo as the SO-101 arm. |
| 2 | USB bus-servo adapter for Feetech STS/SCS servos (e.g. Waveshare Bus Servo Adapter (A)) | 1 | $10–15 | Pi USB to servo bus. Also distributes servo power. |
| 3 | 3S Li-ion pack, 18650 cells, **3S2P with BMS** (11.1 V nominal, 12.6 V full), XT60 | 1 | $30–40 | About 75 × 56 × 40 mm, sits inside the tray. Its 12.6 V maximum is inside the servo's rated range (up to 14 V). The built-in BMS makes it safer than a bare LiPo. |
| 4 | 3S Li-ion charger, 12.6 V 2 A | 1 | $15 | |
| 5 | DC-DC buck converter, **5 V 5 A**, USB-C output | 1 | $12–20 | Powers the Pi 5 |
| 6 | 15 A blade fuse + holder, switch, XT60 pigtail, 18 AWG and 22 AWG silicone wire | – | $15 | |
| 7 | Raspberry Pi **Camera Module 3** (Wide recommended) | 1 | $35 | Goes in the head |
| 8 | **Pi 5 camera cable, 22-pin to 15-pin, 300 mm** | 1 | $4 | Longer than v1's, to leave slack for the moving neck |
| 9 | VL53L1X time-of-flight sensor breakout | 1 | $10–15 | Head, under the camera |
| 10 | MAX98357A I²S amplifier + 28–40 mm 4 Ω speaker | 1 | $9 | Speaker under the head's top grille |
| 11 | USB mini microphone | 1 | $8 | |
| 12 | Raspberry Pi 5 Active Cooler | 1 | $5 | |
| 13 | 3-pin servo-bus extension cables (assorted lengths) | 1 pack | $10 | Daisy chain: body → each leg dock → pitch → knee |
| 14 | Magnetic pogo-pin connector pairs, 4-pin | 4 | $12 | Blind-mate leg docks (optional in a first build: plug the leg cable in by hand instead) |
| 15 | **M3 × 8 socket-head screws: the only fastener** | 100 | $8 | 88 are used: 17 per leg module, 16 for the neck, 4 for the deck |
| 16 | microSD 32 GB+ (if needed) | 1 | $8 | |

## Filament

| Material | Amount | For |
|---|---|---|
| **PETG** | ~930 g (**get two 1 kg spools**) | Robot ~863 g + leg jig ~67 g |
| **TPU 95A** | ~15 g | 4 feet |

## Optional

- **Raspberry Pi AI HAT+** (~$70–110): faster object detection (not supported in the software yet).
- **MPU-6050 IMU** (~$3): the balance code exists, but in the simulation leveling didn't help the trot, so it's off by default.
- **5 mm LEDs** for the eyes.

**Rough total without the Pi: $410–480.** The servos are about half of it. That's more than v1, because each smart servo costs more than a PWM servo. In return there's no driver board, no big power converter, no inserts and no manual calibration.
