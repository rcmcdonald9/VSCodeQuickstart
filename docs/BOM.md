# Parts list (BOM)

Prices are rough USD estimates. Check current prices before you buy. You already have the **Raspberry Pi 5 (8 GB)** and the **Bambu Lab P2S**.

## Must-have

| # | Part | Qty | ~Price | Notes |
|---|------|-----|--------|-------|
| 1 | **25 kg·cm digital servo, standard size, 180°** (e.g. DS3225MG; DS3218 also fits) | 12 | $130–170 | 40 × 20 × 40.5 mm case. Get the **180°** version (or set `actuation_range: 270` in config if you buy 270°). Most come with a 25T aluminium disc horn. Check that yours did. |
| 2 | PCA9685 16-channel PWM/servo driver board (I²C) | 1 | $6 | Adafruit or clone |
| 3 | 3S LiPo 11.1 V, 2200 mAh, XT60 | 1 | $25 | About 105 × 34 × 25 mm. It hangs under the chassis on a strap. |
| 4 | LiPo balance charger | 1 | $30 | Skip if you already have one |
| 5 | LiPo low-voltage alarm (beeper) | 1 | $3 | Plugs into the balance lead. **Don't run a LiPo flat.** |
| 6 | DC-DC buck converter **20 A** adjustable (e.g. "300 W 20 A CC/CV") | 1 | $12 | Servo rail, set to **6.8 V** |
| 7 | DC-DC buck converter **5 V 5 A** with USB-C out (or Pololu D36V50F5) | 1 | $12–20 | Powers the Pi 5 |
| 8 | Raspberry Pi **Camera Module 3** (Wide recommended) | 1 | $35 | |
| 9 | **Pi 5 camera cable** (22-pin to 15-pin, 200 mm) | 1 | $3 | The Pi 5 uses the smaller connector. The cable that comes with the camera will not fit. |
| 10 | VL53L1X time-of-flight distance sensor breakout | 1 | $10–15 | Obstacle stop. Any I²C VL53L1X board. |
| 11 | MAX98357A I²S amplifier board | 1 | $6 | |
| 12 | 40 mm 4 Ω 3 W speaker | 1 | $3 | |
| 13 | USB mini microphone (plug-style) | 1 | $8 | |
| 14 | Raspberry Pi 5 **Active Cooler** | 1 | $5 | Vision and voice keep the CPU busy |
| 15 | XT60 pigtails, 16 AWG + 22 AWG silicone wire, 20 A blade fuse + holder, 20 A switch (or XT60 loop key) | – | $15 | |
| 16 | Wago 221 lever nuts (5-way) | 4 | $6 | Servo power distribution |
| 17 | Velcro battery strap, 20 mm | 1 | $3 | |
| 18 | microSD 32 GB+ (if you don't have one) | 1 | $8 | Raspberry Pi OS 64-bit |

## Fasteners

| Part | Qty | Used for |
|------|-----|----------|
| M3 brass heat-set inserts (M3 × 5.7, OD ≈ 4.6 mm) | 100-pack | Servo ears (48), lid posts (4), head (4) |
| M3 × 10 socket head | 60 | Servo ears into inserts, horn screws on the 6 mm leg plates, lid |
| M3 × 8 socket head | 30 | Horn screws on the hip housing (5 mm plate), head |
| M3 × 25 + M3 nyloc nuts | 4 | Foot cross-bolts |
| M2.5 × 10 brass standoffs + M2.5 screws/nuts | 4 sets | Pi 5 on the lid |

## Filament

| Material | Amount | Parts |
|----------|--------|-------|
| **PETG** (or PLA for a quick first build) | ~400 g | Everything structural |
| **TPU 95A** | ~20 g | 4 feet |

## Optional upgrades

- **Raspberry Pi AI HAT+ (13 or 26 TOPS)** (~$70–110): runs the detector on a Hailo chip, which is much faster than the Pi CPU. The software runs YOLO on the CPU by default. The HAT is a later upgrade.
- **MPU-6050 or BNO085 IMU** (~$3–20): tilt sensing / fall detection (`sensors.imu`).
- 5 mm LEDs for the "eyes" (holes are in the head).

**Rough total without the Pi: $330–420.** The 12 servos are most of it.
