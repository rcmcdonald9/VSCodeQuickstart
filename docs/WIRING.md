# Wiring (v2)

> ⚠️ **Battery safety:** Use a 3S Li-ion pack **with a BMS** and a proper 12.6 V charger. Keep the fuse close to the battery. Unplug the battery when the dog isn't in use.

## Power

```
 3S2P Li-ion (11.1 V nom / 12.6 V full, with BMS)
   └─XT60─► [15 A fuse] ─► [switch] ─┬─► bus-servo adapter POWER IN ─► servo bus (12 V to all 10 servos)
                                      └─► 5 V 5 A buck (USB-C) ─► Raspberry Pi 5
```

- **The servos run straight from the battery.** The STS3215 12 V version is rated up to 14 V, and a 3S pack peaks at 12.6 V, so no big converter is needed.
- **Check the buck output with a multimeter before connecting the Pi:** 5.1 V.
- Wire sizes: 18 AWG from the battery to the adapter and the buck, 22 AWG for the rest.
- **Current, from the simulation:** about 2.5 A standing and 5–7 A walking or turning, so a 15 A fuse leaves headroom.

## Servo bus: one daisy chain, IDs instead of channels

Every servo sits on the same 3-wire bus (power, ground, data). Each has two identical ports, so you just chain them:

```
adapter ─► body harness ─┬─► FR dock ─► FR.pitch (1) ─► FR.knee (2)
                         ├─► FL dock ─► FL.pitch (3) ─► FL.knee (4)
                         ├─► RR dock ─► RR.pitch (5) ─► RR.knee (6)
                         ├─► RL dock ─► RL.pitch (7) ─► RL.knee (8)
                         └─► head.pan (9) ─► head.tilt (10)
adapter USB ─► Pi 5 USB port (/dev/ttyACM0 or /dev/ttyUSB0; set servos.port in config.yaml)
```

| Joint | Servo ID |
|---|---|
| FR pitch / knee | 1 / 2 |
| FL pitch / knee | 3 / 4 |
| RR pitch / knee | 5 / 6 |
| RL pitch / knee | 7 / 8 |
| Head pan / tilt | 9 / 10 |

Servos ship with ID 1. **Before assembly**, give each its ID with only that one servo connected (`python -m robodog.calibrate --set-id N`) and label it.

**Leg docks:** a 4-pin magnetic pogo connector sits in each tray socket's inboard wall, with matching pads on the hip block. When the leg drops in, the connection is made. For a first build you can skip the pogo pins and plug one cable per leg by hand. The pockets are there either way.

**Cables inside each leg:** the dock goes to the pitch servo's port A, and port B goes to the knee servo. The knee cable crosses the hip joint, so leave a loop close to the hip axis.

## Sensors and audio (Pi 5 GPIO)

| Device | Pin | Pi 5 pin (physical) |
|---|---|---|
| VL53L1X (head) | VIN / GND / SDA / SCL | 3.3 V (1) / GND (6) / GPIO2 (3) / GPIO3 (5) |
| MAX98357A (head) | VIN / GND | 5 V (2) / GND (14) |
| | BCLK / LRC / DIN | GPIO18 (12) / GPIO19 (35) / GPIO21 (40) |
| Camera Module 3 (head) | CSI | CAM/DISP 0, **300 mm** cable |
| USB mic | USB | any USB port |

The head turns ±60° and nods ±25°. Route its four cables (camera ribbon, ToF, speaker, mic) through the deck's head-cable slot and leave a slack loop around the neck. Turn the head through its full range by hand once to check nothing pulls.
