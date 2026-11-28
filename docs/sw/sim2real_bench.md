# Sim2real on the M1 bench: bus realism, actuator sysid, determinism (2026-11-27)

This round applies the lessons of Menlo Research's Asimov zero-shot sim2real write-up to the M1 Step-1 bring-up:

- make the hardware one deterministic plant (hour 20 = hour 1);
- treat CAN as noisy, lossy and late;
- run PREEMPT_RT and check for thermal throttling;
- model the actuators instead of randomising around them, and randomise only what is actually random.

Everything here runs on the virtual bench, so it can be checked before anything is bought. The hardware numbers come
from Step 1.

## 1. Bus realism (sw/bus_sim.py, sw/bus_realism.py → [bus_realism.md](bus_realism.md), legacy run [bus_realism_legacy.md](bus_realism_legacy.md))

**Model.** The virtual CAN applies the following, per frame and in both directions:

- latency: fixed + jitter + tail, FIFO-ordered, so a long delay followed by a short one bunches two frames into one cycle;
- dropped frames;
- duplicated frames;
- CRC-rejected frames, which move the TEC/REC error counters and can reach error-passive and bus-off;
- app-level corrupted payloads: a bit flipped after the CRC check, i.e. a buggy controller driver;
- forced bus-off.

Commands act when they arrive within the 2 ms physics step, and replies leave when the request arrived. The first
version charged a sub-cycle delay two whole cycles each way: a 0.4 ms bus gave a 6 ms round trip.

**Profiles:**

| Profile | Represents |
|---|---|
| ideal | the previous behaviour |
| usb | a healthy gs_usb adapter: ≈ 0.4 ms mean, rare 4 ms stalls |
| pil9 | 0–9 ms uniform each way; the Asimov PIL figure, read harshly: round trip up to 20 ms |
| lossy | 1 % lost, 0.5 % duplicated, 0.5 % CRC errors |
| corrupt | 0.2 % corrupted payloads |

**Before the fixes** (legacy: stale-feedback bound 10 ms, no frame gate):

| Profile | Result | Kind |
|---|---|---|
| ideal, usb, lossy | pass | — |
| pil9 | stopped in the safety step's healthy idle (15/100 heartbeats, "feedback missing, NaN or stale") | **false trip** (normal jitter) |
| corrupt | calibrate stopped on "tracking error" from one corrupted position bit | trips on a real fault, but at a single frame |

Further bugs, found by instrumenting the failing runs:

1. **Stale buffered frame accepted as an answer.** After a pause, the newest *queued* frame (status "enabled") passed
   as the reply to a status request, so the CAN-timeout check failed falsely on the lossy bus. Frames were stamped when
   the loop read them, not when they arrived.
2. **CLEAR_ERROR still in flight.** A status that left the drive before the clear arrived was read as a drive fault by
   the next step (pil9, calibrate).
3. **Register access blocked the loop with no heartbeat.** A lost reply means 3 × 50 ms with no heartbeat. On hardware
   that opens the 25 ms watchdog in the middle of calibrate.

**Fixes:**

- `fb_max_age_s` **10 → 20 ms** (bringup.yaml, `safety.FB_MAX_AGE`). Measured on pil9: feedback age p99.9 12 ms, max 16 ms
  in normal running; usb max 4 ms.
- Status requests discard older frames first; real buses stamp frames with the kernel RX time.
- `clear_errors()` waits for an OK status.
- Register waits run control cycles (`TableDriver.idle` → `BringUp._reg_idle`).
- Enable and fresh-feedback waits tolerate a late reply.
- Driver **plausibility gate**: a position, velocity or temperature jump that no motion could produce, compared with the
  last accepted frame, is dropped as a lost frame; an error status must appear on two frames; re-anchor after 3 rejects.
- A torque check in the gate was tried and **reverted**: it rejected 10 good frames on a clean bus (torque legitimately
  jumps at enable and gain switches) and still let small torque corruptions through.

**After the fixes**, all five profiles pass every bring-up step (sysid included), and 20 s of 0.5 Hz ±20° motion gives
**0 supervisor stops**. The observer peak / threshold is 0.40 ideal, 0.41 usb, 0.53 pil9 and 0.43 lossy, with no collision
false trip. On **corrupt** there is **1 collision false trip** (peak 3.4×), caused by a corrupted torque value: a
compliant hold, not a drop. In an earlier run of the same code the peak was 1.06× with no trip.

Real faults are still caught:

| Fault | Result |
|---|---|
| contact at 2 × the threshold | trips in 14–18 ms |
| bus silent | supervisor stop in 22–30 ms (frames already in flight keep the feedback fresh for up to 9 ms); the drive's own CAN timeout fires at 32–38 ms |
| bus-off | stop in 22 ms; the adapter reports BUS-OFF and counts the restart |

**Open: corrupted torque values.** They are not gated. In the corrupt run they raised the suggested collision threshold
6× (0.30 → 1.82 N·m), because they land in the gravity step's noise σ, and they bring the observer close to a false trip
(compliant hold, not a drop). At 0.2 % corrupted frames that is about one false collision trip per 20 s. They must be fixed at the source (the
interface: [can_interface_note.md](can_interface_note.md)), and the soak test fails on any implausible frame.

## 2. Actuator sysid, bring-up step 7 (sw/sysid.py → [sysid_rehearsal.md](sysid_rehearsal.md), [sysid_rehearsal_pil9.md](sysid_rehearsal_pil9.md))

The step runs after "gravity", on the Step-1 joints (`bringup.yaml sysid.joints: [yaw, wpitch]`), at low torque with
the supervisor in the loop:

1. **Friction:** ± constant-velocity sweeps at 4 speeds, centred where gravity loads the joint most so the gear stays on
   one flank. Samples are selected by *measured* position and steady speed, giving Coulomb + viscous from a line fit.
2. **Effective torque constant:** the 20 %-speed sweep is repeated with a known test mass (the operator hangs it while
   the loop keeps holding). The joint is sliding at the same speed, so the friction is the same in both runs, and the
   mass's torque comes from kinematics alone.
3. **Backlash:** a ±2.5° triangle at the zero-gravity angle. The motor travel while |τ| < ½ F_c at each reversal is the
   backlash (the encoder is on the motor side).
4. **Latency:** 30 random-phase torque steps, plus a 1–20 Hz chirp cross-correlation.

The fit is written to `sw/config/actuators_<serial>.yaml` on the bench (a virtual run writes under sw/out). Readers:

- `rnd/twin.py actuator_model()`: the datasheet model is the fallback;
- `sw/bench_env.py`: joint damping and armature;
- `sw/virtual_arm.py`: plant friction, Kt, backlash.

**Superseded 2026-11-28 (Fable review C1):** the table below rehearsed the ARM (the virtual GIM yaw stage at 2:1
turning the forearm, upper arm clamped), which is not the Step-1 machine. `python -m sw.rehearse_step1` now rehearses
the **lever rig** (`--rig step1`: bare motors on 0.27 m levers, ratio 1.0, rubber stops at −15° / +100°, K from the
weighed lever); its current numbers are in [sysid_rehearsal.md](sysid_rehearsal.md) and [../M1_RIG.md](../M1_RIG.md) §2.

**Rehearsal on the arm fixture (historical).** The virtual GIM yaw stage turns the forearm in a vertical plane, next to
the DM-J4310. Hidden values: yaw F_c 0.32, B 0.85, K 0.955, backlash 0.30°; wpitch F_c 0.055, B 0.42, K 1.035,
backlash 0.25°.

| Bus | yaw F_c | yaw B | yaw K | yaw backlash | wpitch F_c | wpitch B | wpitch K | wpitch backlash | latency |
|---|---|---|---|---|---|---|---|---|---|
| usb | +4.7 % | −6.0 % | −1.1 % | −5.3 % | −1.5 % | +0.1 % | −0.1 % | +4.9 % | −1.2 / +1.8 % |
| pil9 | +4.7 % | +0.1 % | −1.0 % | −5.3 % | +2.7 % | +3.7 % | −0.2 % | −3.8 % | −10.9 / −8.3 % (12.3 ms true) |

Two first attempts failed and taught something that applies to the real bench:

- **K from static holds was biased by 8–14 %:** a held joint stops anywhere inside its stiction band.
- **Sweeping through the zero-gravity angle flips the load across the backlash mid-sweep.** The torque glitches
  (yaw viscous −19 %).

**Not randomised once measured** (Asimov lesson 4), per serial:

- Coulomb and viscous friction;
- torque constant;
- backlash;
- the latency distribution: the measured histogram, not 0–9 ms uniform;
- encoder zero;
- the drive torque clamp;
- link masses (CAD + the gravity residual).

If the soak shows drift, model the dependence (e.g. on temperature); do not widen a range. **Still randomised:**

- payload mass and COM;
- contact friction (object, table, finger pads);
- object pose and size;
- lighting and camera pose;
- sensor noise at the measured σ.

**Signals:** joint velocity comes from the drive, and no integrated signal is used. The policy should see only what the
drives and cameras natively report.

## 3. Determinism in the bring-up report (sw/determinism.py) and the soak test (sw/soak.py)

Every bring-up report now ends with a determinism section:

- **loop-period histogram** of every control cycle (same bins and percentiles as sw/loop_jitter.py, which now writes the
  histogram too). SW1 requires p99 |period − 2 ms| < 1 ms;
- **CPU temperature and `vcgencmd get_throttled` flags** at the start and end. A flag that appears during the run fails;
- **PREEMPT_RT status**, scheduling policy, isolated CPUs and cpufreq governor;
- **CAN controller health** per bus (state, TEC/REC, bus-off, restarts, bus errors, RX drops), from SocketCAN
  `ip -details -statistics -json` or the virtual bus.

Off the Pi these are reported as informational and skipped gracefully. This container: PREEMPT_DYNAMIC (not RT), no
thermal sensor, no vcgencmd.

**Soak test** (Step-1 exit criterion, docs/M1_BENCH_ARM.md and M1_READINESS 7): sysid → 2 h of 0.25 Hz cyclic motion at
≈ 50–60 % rated → sysid again. On the Step-1 lever rig (`--rig step1 --load soak`, docs/M1_RIG.md): 4 discs on the
GIM lever (1.04 kg at 0.27 m, 3.0 N·m horizontal), 2 on the DM lever (1.86 N·m), ≈ 55 % RMS; the run fails outside
45–65 %. (The old "2 kg at the 0.3 m lever on the yaw stage" assumed the arm's 2:1 stage: 118 % of the bare motor's rated.)

Pass if all of these hold:

- Coulomb drift ≤ 20 %;
- viscous drift ≤ 25 %;
- K drift ≤ 5 %;
- backlash growth ≤ +0.1°;
- latency mean ≤ +0.5 ms and p95 ≤ +2 ms;
- no supervisor stop, bus-off, new throttle flag or implausible frame;
- jitter p99 < 1 ms on the Pi;
- CPU < 80 °C.

Virtual rehearsal (45–60 s): a stable drive **passes**, with drift Coulomb −0.9 %, K −0.01 %. A drive whose friction
grows 25 %/K of motor temperature **fails** (Coulomb +35 %). See [soak_virtual.md](soak_virtual.md) and
[soak_virtual_drift.md](soak_virtual_drift.md) (the arm, historical); on the lever rig
[soak_virtual_step1.md](soak_virtual_step1.md) and [soak_virtual_step1_drift.md](soak_virtual_step1_drift.md).

## 4. CAN interface

Buy both for Step 1: the two gs_usb UCANs plus one Waveshare 2-CH CAN FD HAT ($46.99), and measure them with
`sw/can_latency.py`. See [can_interface_note.md](can_interface_note.md).
