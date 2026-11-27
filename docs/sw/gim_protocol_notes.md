# SteadyWin GIM CAN protocol: what is known, what is not (2026-11-27)

Fable review M4: the safety concept (sw/safety.py total-torque bound kp·e + kd·ė + τ_ff, torque feedback for the
collision observer, a drive-side CAN timeout, a torque limit saved to flash) assumes an MIT / impedance mode on **three
different SteadyWin drivers** in one arm. If one of them only offers position or velocity modes, the supervisor's
torque bound does not apply to that joint. This note collects what is public. The maker's own manual host is blocked
from this environment and was not worked around. The machine-readable form is `GIM_KNOWN` / `GIM_DRIVERS` in
[sw/drivers/gim.py](../../sw/drivers/gim.py); `python -m sw.drivers.gim` prints every TODO field with its status.

## Sources found (search snippets only; the documents themselves could not be opened from here)

| Source | What it says | Weight |
|---|---|---|
| SteadyWin GIM6010-8 manual rev 1.2 (public copy on a personal GitHub Pages site, sdalal1.github.io) | Has a "CAN protocol" section; release note 1.2, 2024-05-24: "Fixed bug in CAN MIT protocol" | An MIT mode exists on the GIM6010-8's driver, firmware-dependent |
| SteadyWin GIM GUI manual v2.0 (Chinese; in NVIDIA GEAR lab's GEAR_Neck firmware folder, research.nvidia.com) | Control modes: position, speed, current, MIT ("目前支持位置、速度、电流、MIT四种控制模式"); an "MIT protocol test demo"; the tool scans IDs 1–252, IDs must be unique | MIT is a selectable mode, not necessarily the default |
| manuals.plus, GIM8108-6 manual | Two driver options (an MIT driver and an "SHS" driver, different nominal voltages); default CAN is the "SmartCAN protocol, speed 1M" | Protocol depends on the driver variant; 1 Mbit/s |
| Arduino forum, "Interfacing my GIM 8115-9 actuator…" | MIT-style frames did not move the motor; the owner suspected SteadyWin's own protocol, enabled motor mode only by changing firmware settings over a serial terminal; quotes an enter frame 0x91 00…00 from another document | Expect to configure the mode in the vendor tool first |
| Community Google Doc on the "SteadyWin V3" (MIT Cheetah / HobbyKing lineage) + Ben Katz's repos | Enter FF×7 FC, exit FF×7 FD, zero FF×7 FE; 8-byte MIT command / 6-byte reply | The candidate frames `sw/gim_discover.py` probes |
| openelab reseller page, SteadyWin driver board | GDS6 = SDC102, **GDS68 = SDC104** (CAN only, ≤ 48 V, 15 A), **GDZ468 = SDC215** (CAN + RS-485, 12–40 V, 10 A); encoders MT6816 / MA732 | Driver naming; motor-side single-turn encoders (output zero not absolute through 8:1) |
| cp-legged-robots docs, GIM8108 page | Points to SteadyWin's GIM8108 Google Doc and Ben Katz's motor-drive documentation | Same family |

Nothing public was found for the **SDC303** (GDM810 / GDK810, the inline GIM8108-36 roll), for a **CAN-command
timeout** parameter, for a **current / torque limit saved to flash**, or for **status / error codes** on any of them.

## Field by field

| Table field | Status | What we have |
|---|---|---|
| Bit rate | candidate | 1 Mbit/s ("SmartCAN, speed 1M") |
| CODEC (MIT layout) | candidate | MIT mode in the GIM6010-8 manual and the GUI; may need selecting in the tool |
| ENABLE / DISABLE / SET_ZERO | candidate | FF×7 FC / FD / FE (Ben Katz family); 0x91… quoted for a GIM8115-9 |
| CONTROL / FEEDBACK IDs | candidate / unknown | command on the node ID; reply ID unknown (Katz firmware answers on 0) |
| RANGES (P, V, T, kp, kd) | unknown | firmware settings; the probe decodes under P 12.5 / 95.5, V 30 / 45 / 65, T 10 / 18 / 54 |
| Node ID / feedback ID registers | unknown | IDs 1–252 via the vendor tool |
| CAN timeout | **unknown** | no source; the safety step of sw/bringup.py proves or disproves it on the bench |
| Torque / current limit in flash | **unknown** | no source; the limits + safety steps prove it (write, read back, power cycle) |
| Status / error codes | unknown | — |
| ABSOLUTE (zero survives power-off) | assumed False | motor-side MT6816 / MA732 encoders: re-zero or use the AS5048A joint encoders |
| Same protocol on SDC104, SDC303, the GIM8108-8's driver? | **unknown** | seller question 5 (M1_BENCH_ARM.md) |

## What changes because of this

- **M1 kit Step 1 buys one GIM6010-8 only** (with the DM-J4310, servos, CAN adapters and the safety parts). No further
  GIM is bought until Step 1's exit criteria are met (docs/M1_READINESS.md).
- **Bring-up step 1 is protocol discovery** on that one motor: `python -m sw.gim_discover --bus socketcan:can0 --id <n>`
  (listen; the no-torque exit-motor-mode probe; with `--enable`, a zero-gain motor-mode trace while the output is turned
  by hand). The motor is alone on the bus, unloaded in its fixture, the supply limited to 1 A: on an unknown protocol any
  8-byte frame could mean something else. Rehearsal: `python -m sw.gim_discover --bus virtual --enable` (sw/test_s2.py).
- If the SDC104 has no usable impedance mode, the GIM joints need a different safety argument (drive-side current limit
  + position mode + a software torque estimate) before the arm is built: a design stop, recorded in the bring-up report.
- The GIM8108-36 (SDC303) is a different driver: the discovery runs again on it when it arrives (Step 2), before it is
  mounted in the shoulder.
