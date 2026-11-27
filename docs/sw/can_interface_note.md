# CAN interface decision note: gs_usb USB adapters vs an MCP2518FD SPI HAT on the Pi 5 (2026-11-27, no purchase made)

**Why this note exists:** Menlo Research's Asimov sim2real write-up says they had to fix CAN corruption and latency, and
that they moved from a buggy native CAN path to SPI-CAN on an **MCP2518FD** (large TX/RX buffers, replacing an MCP2515). The
loop-47 cost pass chose **candleLight / gs_usb USB-CAN adapters** for Pip (FYSETC UCAN, STM32F072, $19 each,
rnd/electrical.py A14; it replaced the Waveshare 2-channel SPI HAT that was in the plan before). This note checks that
choice against the Asimov lesson before the Step-1 order.

**Recommendation for Step 1: buy both.** Keep the two FYSETC UCAN adapters ($38) **and add one Waveshare 2-CH CAN FD HAT
($46.99)**. Step 1 is the cheapest moment to measure the two on our own Pi 5, our own drives and our own loop, and the
answer decides five channels for the full robot (A14). The extra cost is $47 ($54 with 15 % contingency).

## Comparison

"Sourced" means a link below; everything else is reasoning from how the parts work, to be replaced by the Step-1
measurement (`python -m sw.can_latency`).

| | candleLight / gs_usb (FYSETC UCAN, STM32F072) | MCP2518FD SPI HAT (Waveshare 2-CH CAN FD HAT) |
|---|---|---|
| Path to the CPU | CAN → bxCAN → firmware → USB full speed (bulk endpoint) → RP1 xHCI → `gs_usb` driver → SocketCAN | CAN → MCP2518FD → SPI (RP1) → `mcp251xfd` driver (threaded IRQ) → SocketCAN |
| Latency (expected) | USB full speed runs on 1 ms frames: RX latency ≈ 0.1–1 ms, plus host-controller scheduling; TX similar | IRQ → threaded handler → about 2–5 SPI transfers per received frame (linux-can thread: 5 transfers per RX frame, 2 with the performance governor), ≈ 50–200 µs when the IRQ thread runs on time |
| Jitter (expected) | Bounded below by the 1 ms USB frame; tails come from xHCI / USB-hub stalls and CPU frequency steps | IRQ-thread scheduling: tens of µs on PREEMPT_RT with a raised IRQ-thread priority, ms tails under load on a stock kernel. One report of ≈ 6 ms TX jitter with same-ID frames on another SoC (NVIDIA forum) |
| Buffering | bxCAN: 3 TX mailboxes, 2 RX FIFOs × 3 frames in hardware; the firmware queues frames in RAM towards USB (reference manual / firmware; not sourced here) | 2 KB message RAM, 31 configurable TX/RX FIFOs + a TX queue + a TX event FIFO, 32 filters (sourced). The old MCP2515 has 2 RX + 3 TX buffers (sourced): at our ≈ 3,500 frames/s per bus a late IRQ overruns it, which matches the Asimov problem |
| Bus-off / error handling | Error frames and state forwarded to SocketCAN; restart through `ip link … restart-ms`. Firmware-dependent: test it | Full SocketCAN error state, TEC/REC, `restart-ms`; mature in the driver |
| HW timestamps | Since the 2023 gs_usb patches (Pengutronix); stamped in the firmware's CAN RX interrupt, better than host time, not true hardware stamps (sourced) | Controller timestamps (TBC), stamped before the SPI path |
| Linux driver maturity | `gs_usb` in mainline for about 10 years; candleLight firmware widely used | `mcp251xfd` in mainline since 5.10 (2020), maintained by Pengutronix. **Pi 5 reports:** "DMA transaction timed out" / "SPI transfer failed" on kernel 6.1.58 (Seeed forum); errors with a second MCP251xFD interrupt on 6.12 (RevPi forum). The dual-channel case is exactly the Waveshare HAT (sourced) |
| Isolation | Not isolated (UCAN) | Isolated, with protection on the bus side (Waveshare listing) |
| CAN FD | No (F072 is classic CAN); FD needs a G0/G4 candleLight-FD board | Yes (not needed today: DM-J4310 and, as far as known, the GIMs are classic CAN) |
| Pi resources | 2 USB ports (or a hub), no GPIO | GPIO header: SPI + 2 interrupt GPIOs. **Check that the HAT's INT pins do not collide with GPIO17 / GPIO27** (heartbeat / contactor aux, `--estop gpiod:gpiochip0:17:27`) |
| Price | $19 each, $38 for M1's two buses (MatterHackers, rnd/m1_kit.py) | $46.99 for both channels (waveshare.com SKU 17075, sourced); The Pi Hut £45.20, sold out (sourced) |
| Robot scale (A14: 5 channels) | 5 × $19 = $95 | 2 HATs do not stack on SPI0 without rework; 1 HAT (arms) + USB for the base (the pre-loop-47 plan) |

## What the virtual bench already says (docs/sw/bus_realism.md)

- The safety chain does not need sub-millisecond CAN. With the fixes from this round, the bring-up, supervisor and
  collision observer pass on a bus with **0–9 ms delay each way** (pil9 profile). The cost is control quality: the round
  trip goes from 2 ms to a mean of 12 ms. A typical healthy USB adapter (usb profile) adds almost nothing (mean 2.03 ms,
  max 6 ms).
- **What matters is corruption and loss, not the mean latency.** Corrupted payloads are why Asimov changed controllers.
  The driver's new plausibility gate turns most of them into lost frames, but a corrupted *torque* value still reaches
  the collision observer (one false collision trip in 20 s at 0.2 % corruption, a compliant hold), and corrupted frames
  during the gravity step raised the suggested collision threshold 6× (0.30 → 1.82 N·m). The interface must deliver **zero** implausible frames in the soak
  test. That is a pass/fail criterion, not something to filter around.

## Step-1 measurement plan (decides the robot's 5 channels)

1. Wire one UCAN and one HAT channel to the Step-1 B2 cable (DM-J4310 on it, 2 × 120 Ω at the ends).
2. `python -m sw.can_latency --pair can0,can2 --seconds 600` both ways; then `--drive can0 --id 5` and `--drive can2 --id 5`
   (status-request round trip at 500 Hz). Repeat with `--stress 3` (cameras or busy loops) and on PREEMPT_RT.
3. Bring-up and a 30 min soak (`python -m sw.soak --hours 0.5`) on each interface. Compare the determinism section:
   loop jitter, CAN error counters, bus-off events and implausible frames.
4. **Pick the HAT for the arm buses if** its round-trip p99.9 is under 1 ms, it loses no frames, it has no RX overrun and
   no driver error in dmesg over the soak, on the Pi 5 kernel we ship. **Otherwise stay on gs_usb** (cheaper, simpler,
   no GPIO conflict). Either way, the measured latency histogram goes into the actuator model (sysid `latency_ms`) and
   into sw/bus_sim.py as the "measured" profile. Do not randomise latency 0–9 ms.

## Kit change (for the lead: rnd/m1_kit.py, Step 1; not edited here)

Add one line after the USB-CAN adapter line (`("Electronics (A14)", "USB-CAN adapter, candleLight / gs_usb …", 2, 19.0, …)`):

```python
        ("Electronics (A14)", "Waveshare 2-CH CAN FD HAT (2 × MCP2518FD, isolated, SKU 17075): SPI-CAN alternative, measured "
         "against the two gs_usb adapters on the Step-1 bus (docs/sw/can_interface_note.md)", 1, 46.99,
         "decides the robot's 5 CAN channels (A14): latency, jitter, loss, bus-off on the Pi 5; check its INT GPIOs vs 17/27",
         "waveshare.com $46.99 (2026-11-27)"),
```

Step 1: $1,189 → **$1,236** before contingency, $1,368 → **$1,421** with 15 %.

## Sources

- [Waveshare 2-CH CAN FD HAT](https://www.waveshare.com/2-CH-CAN-FD-HAT.htm) ($46.99, MCP2518FD, isolated) · [The Pi Hut listing](https://thepihut.com/products/2-channel-isolated-can-fd-expansion-hat-for-raspberry-pi) · [Waveshare wiki](https://waveshare.com/wiki/2-CH_CAN_FD_HAT)
- [MCP2518FD datasheet](https://cdn.sparkfun.com/assets/8/a/c/b/5/MCP2518FD_datasheet.pdf) (31 FIFOs, TX queue, TEF, 32 filters) · [MCP2515 datasheet DS20001801K](https://cdn.sparkfun.com/assets/1/0/e/4/0/MCP2515-Family-Data-Sheet-DS20001801K.pdf)
- linux-can on mcp251xfd SPI transfers per frame and busy polling: [lore.kernel.org thread](https://lore.kernel.org/all/20220117150833.d4e4f4dhjvsvmxkt@pengutronix.de/t/), [Kleine-Budde](https://lore.kernel.org/linux-can/20210615071557.o7fjkleuk777otvm@pengutronix.de/)
- Pi 5 reports: [Seeed forum, 2-channel CAN FD shield on the Pi 5](https://forum.seeedstudio.com/t/rpi5-2-channel-can-bus-fd-shield-for-raspberry-pi/274292) · [RevPi forum, Pi 5 CAN with Balena](https://revolutionpi.com/forum/viewtopic.php?p=17473)
- TX jitter report: [NVIDIA forum, MCP2518FD SPI-CAN transmit jitter (~6 ms)](https://forums.developer.nvidia.com/t/mcp2518fd-spi-can-transmit-jitter-6ms-with-same-id-frames/370510)
- gs_usb hardware timestamps: [Pengutronix, "Yes we CAN… add new features" (2023)](https://pengutronix.de/en/blog/2023-01-30-yes-we-can-add-new-features.html) · [candleLight firmware](https://github.com/candle-usb/candleLight_fw)
- Not reachable from this session (blocked hosts, not retried): the CTU CAN latency tester paper (canbus.pages.fel.cvut.cz) and the RTLWS gateway timing papers (static.lwn.net). These hold measured Linux CAN latencies and should be read when the network allows them.
