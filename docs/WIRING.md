# Wiring

> ⚠️ **LiPo safety:** Charge in a LiPo bag, never unattended. Unplug the battery when you're not using the robot. Keep the low-voltage alarm plugged in. Put the fuse as close to the battery as you can.

## Power

```
 3S LiPo ──XT60──► [20 A fuse] ──► [switch] ──┬──► Buck A (20 A, set 6.8 V) ──► Wago V+ / Wago GND ──► 12 servo red/black
 (11.1 V)                                      │                                       │
                                               │                                       └──► PCA9685 V+ terminal (servo rail only)
                                               │
                                               └──► Buck B (5 V 5 A, USB-C) ──► Raspberry Pi 5 USB-C power port
```

- **Set both buck converters with a multimeter before you connect anything.** Buck A goes to 6.8 V. Check your servo's maximum; most 25 kg servos are rated 4.8–6.8 V. Buck B goes to 5.1 V.
- **Servo current doesn't go through the Pi.** A trotting dog pulls 5–10 A in peaks. Run 16 AWG from buck A to two Wago lever nuts (V+ and GND). Cut the red/black wires of each servo extension, or use servo power splitter leads, so each servo gets power straight from the Wagos. The orange signal wires go to the PCA9685. If you'd rather plug all 3 pins into the PCA9685 for simplicity, also solder a 16 AWG pair to the V+/GND rails at the far end of the PCA9685's header rows, so you're not relying on the small screw terminal alone.
- **Common ground:** The battery negative, both buck converter outputs, the PCA9685 GND and the Pi GND must all connect. Over the I²C cable this happens automatically.

## Signals: Raspberry Pi 5 GPIO header

| Device | Device pin | Pi 5 pin (physical) |
|--------|-----------|---------------------|
| **PCA9685** | VCC (logic) | 3.3 V (pin 1) |
| | GND | GND (pin 6) |
| | SDA | GPIO2 / SDA (pin 3) |
| | SCL | GPIO3 / SCL (pin 5) |
| **VL53L1X** (head) | VIN | 3.3 V (pin 17) |
| | GND | GND (pin 9) |
| | SDA / SCL | Same bus as the PCA9685 (daisy-chain or pin 3/5) |
| **MAX98357A** amp | VIN | 5 V (pin 2) |
| | GND | GND (pin 14) |
| | BCLK | GPIO18 (pin 12) |
| | LRC | GPIO19 (pin 35) |
| | DIN | GPIO21 (pin 40) |
| | Speaker +/− | 40 mm speaker (in the head's left cheek) |
| **Camera Module 3** | CSI | CAM/DISP 0 connector (Pi 5 22-pin cable) |
| **USB mic** | USB | Any USB port |
| MPU-6050 (optional) | VCC/GND/SDA/SCL | 3.3 V / GND / I²C bus |

I²C addresses: PCA9685 `0x40`, VL53L1X `0x29`, MPU-6050 `0x68`. After setup, check them with `i2cdetect -y 1`.

## Servo channels (PCA9685)

| Leg | roll (hip sideways) | pitch (hip fore/aft) | knee |
|-----|------|-------|------|
| Front-right **FR** | 0 | 1 | 2 |
| Front-left **FL** | 4 | 5 | 6 |
| Rear-right **RR** | 8 | 9 | 10 |
| Rear-left **RL** | 12 | 13 | 14 |

Channels 3, 7, 11 and 15 are spare (a tail, maybe?). You can remap any channel in `software/config.yaml`.

## Where things go

- **Inside the chassis bay** (between the four roll servos): the PCA9685, both buck converters, the Wago nuts and the fuse.
- **Under the chassis:** the LiPo, held by the velcro strap through the four floor slots. The XT60 lead comes up through the centre hole.
- **On the lid:** the Pi 5 with Active Cooler on M2.5 × 10 standoffs, USB ports facing the rear. The big slot under the Pi carries the I²C and power leads down into the bay.
- **Head:** Camera Module 3 (4 × M2 screws), VL53L1X behind the lower window, speaker in the left cheek, mic hole on top. The camera ribbon runs through the slot under the head.
- **Leg cables:** The pitch and knee servo leads run along the upper leg and enter the chassis through the side-wall holes. Leave a loop at each joint so the leg can move through its full range without pulling on the cable.
