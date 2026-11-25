# T01: Verified specs, summary and design impacts

Full data with a source for every number: [`rnd/specs/actuators.yaml`](../../rnd/specs/actuators.yaml), [`rnd/specs/components.yaml`](../../rnd/specs/components.yaml).

## Key numbers

| Part | Rated / peak torque | Speed | Mass | Interface | Price |
|---|---|---|---|---|---|
| **SteadyWin GIM6010-8** (cartridge M) | 5 / 11 N·m | 120 rpm rated, 420 max | 388 g with driver | CAN + USB-C, integrated driver | $70–95 |
| **Damiao DM-J4310-2EC V1.1** | 3 / 7 N·m | 120 rated, 200 no-load @24 V | 300 g | CAN 1 Mbps, output-absolute encoder | $125–179 |
| Damiao DM-J4340P-2EC (option) | 9 / 27 N·m | 36 rated, 52 no-load @24 V | ~370 g | CAN | $175–187 |
| Damiao DM-J8009P-2EC (heavy shoulder option) | 20 / 40 N·m | 100 @24 V | ~1 kg (estimate) | CAN, crossed-roller output bearing | $395–480 |
| **Feetech STS3215-C018 12 V** (cartridge S) | 0.98 / 2.94 N·m | ~45 rpm | 55 g | TTL bus, 4096-count encoder | $17–22 |
| AndyMark 6" DuraOmni wheel | rated 120 lb load | – | 0.45 kg | 1.125" bore, 6 holes on a 1.875" bolt circle | $29–37.50 |
| 6S 24 V 10 Ah Li-ion pack | 222 Wh | – | ~1.5 kg | – | estimate $80–140 |
| Orbbec Gemini 336 (head depth camera) | 0.10–20 m | – | 99 g | USB3-C | ~$310 |
| RealSense D405 (wrist depth, option) | 0.07–0.5 m | – | 40–58 g | USB3 | ~$419 |
| RPLIDAR C1 | 12 m, 0.05 m blind spot | – | 110–180 g | official ROS 2 driver | $69–80 |
| PAA5100JE optical flow | 15–35 mm working height | – | – | SPI | $25–37 |
| Closed-loop NEMA 23, 2 N·m (kit with driver) | – | – | – | step/dir | ~$88 |

## Design impacts (decisions for the plan)

1. **The power bus changes from 48 V to 24 V (new D7).** The GIM6010-8 manual rates it at 12–28 V, and the Damiao parts are 24 V variants. The concept's 48 V e-bike battery is out.
   - **New battery: a 6S Li-ion pack** (22.2 V nominal, 25.2 V full) with a BMS.
   - A 10 Ah pack is about **1.5 kg**, against 2.7–4 kg for a 48 V e-bike pack. That's a big win for carrying it upstairs (R3).
   - 222 Wh against R7 runtime gets checked in T02.
2. **The drive works on carpet on paper (R4/R6).** GIM6010-8 straight onto a 6" omni wheel gives:
   - about 66 N of push per wheel at rated torque
   - 0.96 m/s at rated speed and up to about 3.3 m/s at maximum

   That meets the 0.8 m/s target with margin. Torque for going over thresholds gets checked in T03.
3. **The GIM6010-8 is light: 388 g including its driver.** Nine of them (six arm, three base) come to about 3.5 kg. That's good for the carry budget.
4. **The shoulders may need more torque than one GIM6010-8 gives** (5 N·m rated). The plan's printed 2:1 belt plus spring assist stays the default. The proven upgrades are a Damiao DM-J4340P (9 N·m, slow) or DM-J8009P (20 N·m, $400+). T05 decides.
5. **The motors don't need CAN-FD.** The GIM6010-8 and DM-J4310 both use classic CAN at 1 Mbps, so a ~$35 USB-CAN adapter works. candleLight FD (€46) keeps the door open for faster CAN later.
6. **The camera budget was too low.** The concept assumed $30 wrist cameras; good global-shutter ones cost $60–155. Cheap $20–35 rolling-shutter USB cameras are what most LeRobot builds use. The choice is left to the AI phase, and the BOM gets updated in T16.
7. **The lift needs a hold when power is off.** Otherwise the torso drops. Options: a NEMA 23 with a 24 V brake (about $41 for the motor), or a self-locking lead screw. T09 decides.

## Still to verify (when hardware arrives, or from drawings)

- The GIM6010-8 mounting pattern and exact outside dimensions (installation drawing). Cartridge M sleeves wait on this, in T06.
- A real 6S pack listing (price, connector, continuous current: it needs about 30 A peak, estimate).
- How well optical-flow odometry works on *your* carpet (bench test).
