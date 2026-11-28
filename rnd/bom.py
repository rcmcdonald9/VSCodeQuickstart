"""Bill of materials from the CAD quantities.  python -m rnd.bom

Prices: actuators and sensors from rnd/specs/*.yaml (sourced); hardware is ESTIMATE (typical online prices,
single quantity, 2026). The Pi 5 is excluded (the owner has one), as are tools.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

from . import m1_kit as K
from .belts import STAGES, stage
from .budget import actuator_cost
from .params import BASELINE, act

TENSION_RING_SITES = ("elbow", "yaw")   # = rnd/parts/belt_stage.TENSION_RING_SITES (not imported: build123d is not needed for the BOM)

DOCS = Path(__file__).resolve().parent.parent / "docs" / "rnd"


def items(d=BASELINE, timer=None):
    """BOM lines [(category, item, qty, unit $, source)]; timer = the U20 base OFF-delay timer option (PIP_BASE_TIMER by default)."""
    names = [n for n, _ in d.arm.joints.values()] * 2 + [d.base.wheel_motor] * 4 + ["STS3215-C018"] * 2 + [d.upper.lift_motor]
    counts = {n: names.count(n) for n in sorted(set(names))}
    belts = [(j, round(stage(j, r, z, c, w)["length"] / 5) * 5) for j, r, z, c, w in STAGES]
    pp = json.loads((DOCS / "print_plan.json").read_text()) if (DOCS / "print_plan.json").exists() else {"kg": 10.4}
    ns = 2 * len(STAGES)                 # belt stages on both arms (D24 inline roll: pitch, yaw, elbow; the roll has none)
    inline = d.arm.roll_drive == "inline"
    I = []
    A = lambda cat, item, q, unit, src="ESTIMATE": I.append((cat, item, q, unit, src))
    for n, q in counts.items():
        if n in BEST_PRICE:                                    # loop 47 cost pass: best verified price, same part
            A("Actuators", n, q, *BEST_PRICE[n])
        else:
            A("Actuators", n, q, act(n)["price"], "actuators.yaml (mid list price)")
    A("Wheels & power", '6" DuraOmni omni wheel', 4, 33.0, "components.yaml")
    A("Wheels & power", "6S 10 Ah 21700 Li-ion pack (Lumenier NAV, no BMS): 35 A cont. with an 80 °C cut-off, 90 A burst, 975 g; re-terminated XT90", 1, 150.0,
      "racedayquads.com listing (loop 13); ratings: RaceDayQuads / GetFPV (2026-11-27 search)")
    A("Wheels & power", "6S smart BMS JK-B2A8S20P-HC: 200 A cont. / 350 A peak, 2 A active balance, CAN + RS485, 2 NTC (Neon #2 F11: the 40 A class BMS tripped on the 74 A peak) "
      "— A13: telemetry for the dock stop and the software pack-current budget", 1, 95.0, "nkon.nl €83.95 (2026-11-27); 18650batterystore $145 (sold out)")
    A("Wheels & power", "Steel battery box for the base bay (pack + BMS, vented) — Neon #2 F11", 1, 20.0, "ESTIMATE")
    A("Wheels & power", "Fuses + holders: 60 A MIDI main (Neon #2 F1/F11: a 40 A main sat at 93 % of the 37 A sustained) + ATO 20 / 20 / 20 / 10 / 7.5 / 5 A", 1, 25.0,
      "ESTIMATE")
    A("Wheels & power", "E-stop (A5 v2): 2 two-channel latching mushrooms + TE ALEV100 DC contactor (24 V coil, aux contact) + switched pre-charge (K_PRE relay + 3.3 Ω RH-50) + SN74LVC1G123 watchdog on its own LDO", 1, 150.0,
      "LEV100A5ANG $92 (Master Electronics); rest ESTIMATE")
    A("Wheels & power", "Regen clamp / bus discharge (A5 v2, Neon #2 F1): 3 × Vishay **NH-50** 0.56 Ω (non-inductive) + 3 × VS-40CPQ060-N3 freewheel Schottky + 3 × IRFB7530 + "
      "TLV3202 / TL431B, gate driver + gate resistors, film caps, 85 °C thermal switch (robot peak regen 2.8 kW; electrical.md)", 1, 3 * K.NH50[0] + K.CLAMP_PARTS,
      "D29 #7: NH-50 NH050R5600FE02 " + K.NH50[1] + "; VS-40CPQ060-N3 RS £37 / 25; rest ESTIMATE")
    A("Wheels & power", "Safety-board logic (netlist v3, electrical.md): discharge one-shot (LVC1G123 on Ā), LVC1G74 latch + power-on reset, LVC1G11 / LVC1G32 / LVC1G04 (K1 hold on AUX), UCC27524, "
      "1 kΩ 3 W bleeder, 12 V buck + 3.3 V LDO, 2 × PC817, 2-layer PCB (the M1 bench board moves to the robot; its ideal diode stays on the bench)", 1, 40.0, "ESTIMATE")
    A("Wheels & power", "Safety-board netlist v3 additions (Astra #3 critical 3, safety_board_sim.md): Q4 100 V logic-level coil driver (was a 30 V part seeing ≈ 52 V), "
      "U16 50 µs minimum-on one-shot, U17 AUX / K1 discrepancy monitor, U18 12 V UVLO; the branch current trip (ACS724-30A + RC + comparator) is the next line",
      1, K.BOARD_V3[0] - K.ACS724[0] + 1.0, K.BOARD_V3[1])
    A("Wheels & power", "Arm-branch current trip U19 (one per arm branch: the 20 A fuse leaves the in-arm 16 AWG unprotected at 18–35 A; electrical.md U19)", 2, *K.ACS724)
    cap_w = "stock Ø90 × 10 round LED heatsinks, faced and drilled (D29 #5)" if K.REAR_CAP == "stock" else "aluminium finned rear caps"
    A("Drive & structure", f"Thermal (Fable M7; thermal.md §3 'the mount the BOM pays for'): {cap_w} on the {ns} arm belt-joint GIM6010 cartridges "
      "(the inline roll runs bare under its cover) + 10 k NTC per GIM; capless, the task-day winding is 102 °C (limit 100), with the cap 68 °C", 1, ns * K.AL_CAP[0] + 6.0,
      "m1_kit AL_CAP: " + K.AL_CAP[1] + "; NTC 10-pack ESTIMATE $6")
    # W15b base channel (electrical.md *Base channel*, netlist Q9 / U20–U27 / Q10 / R7 / R8; docs/sw/base_stop.md items 1–6): the base moves off K1
    # for the category-1 base stop; robot only (the M1 bench has no base)
    tm = timer or K.BASE_TIMER
    A("Wheels & power", "Base channel (W15b) Q9 base switch: TI LM5069-1 hot-swap controller (latch-off; its soft-start is the base pre-charge, 35 A breaker) "
      "+ IRFB7530 + 1 mΩ sense, VIN → VBUS_B, not on K1", 1, *K.Q9_SWITCH)
    A("Wheels & power", "Base channel (W15b) clamp leg Q10 / R7 / D7: 1 × Vishay **NH-50** 0.56 Ω ∥ VS-40CPQ060-N3, IRFB7530, U23 TLV3202 (on 26.0 / off 25.7 V) "
      "+ 2 µs filter, U24 50 µs min-on and the FORCE_B 100 ms one-shot (2 × LVC1G123), U22b UCC27517, passives: holds VBUS_B ≤ 26.3 V while the ramp regenerates",
      1, K.NH50[0] + K.BASE_CLAMP_REST, f"NH-50 NH050R5600FE02 at ${K.NH50[0]:.0f} ESTIMATE (D29 #7, as the arm clamp line); the rest ESTIMATE (DigiKey-class)")
    A("Wheels & power", "Base channel (W15b) logic: U22 base watchdog WD_B (SN74LVC1G123, 25 ms, GPIO23), U26 DISC_B comparator + 50 ms RC (shorted Q9 → BMS "
      "discharge-off request), U27 photoMOS (AQV252G class: LATCH into U20 channel 1), R8 1 kΩ 3 W base bleeder", 1, *K.BASE_LOGIC)
    A("Wheels & power", "Base channel (W15b) S1c/d, S2c/d: 2 more NC contact blocks per mushroom (U20's two channels start from the button itself)", 4, *K.NC_BLOCK)
    A("Wheels & power", "Base channel (W15b) SB1 ARM / reset pushbutton becomes 2-pole (SB1b resets U20): the difference", 1, *K.SB1_2POLE)
    tn, tsrc = K.BASE_TIMERS[tm][1:]
    A("Wheels & power", f"Base channel (W15b) U20 base OFF-delay timer, **pending owner decision** (PIP_BASE_TIMER={tm}, provisional default board; the other "
      f"option's total is in cost_down.md / bom.json): {tn}", 1, K.BASE_TIMERS[tm][0], tsrc)
    A("Wheels & power", "DC-DC 24→12 V 10 A (head servos), 24→5 V 5 A", 1, 30.0)
    A("Wheels & power", "D14 elbow-node power: 24→12 V 6 A buck (two stalled STS3215 draw 5.4 A, Astra #16) + half-duplex TTL transceiver (one per arm)", 2, 10.0)
    A("Wheels & power", "Connectors (Neon #2 F10): XT90 (pack), 5 × XT60 (2 arm detach, 3 base ↔ upper), 2 × 10-pin + 1 × 12-pin pogo blocks (USB on the arm plugs), JST-GH/VH, wire", 1, 62.0)
    A("Wheels & power", "Lift umbilical (Neon #2 F10): drag chain 15 × 30 mm R 50, 0.55 m + 2 × 14 AWG and 3 × 18 AWG silicone pairs, CAN / e-stop / signal cores, shielded USB (1.2 m each)", 1, 30.0)
    A("Compute & sensors", "Raspberry Pi 5 8 GB", 1, 0.0, "owner has one")
    A("Compute & sensors", "USB-CAN adapter, candleLight/gs_usb (FYSETC UCAN, STM32F072): A14's five channels (CAN A base + lift, two per arm); replaces the Waveshare 2-CH HAT + 3 × $35 adapters", 5, 19.0,
      "MatterHackers FYSETC UCAN V1.0 $19 (2026-11)")
    A("Compute & sensors", "Bus-servo USB-TTL adapter (half-duplex, ID programming only: D14 elbow nodes and the torso node drive the TTL buses)", 1, 8.0)
    A("Compute & sensors", "Raspberry Pi 5 Active Cooler (official heatsink + PWM fan): keeps the control loop out of thermal throttling (Menlo sim2real lesson)", 1, 5.0)
    A("Compute & sensors", "Powered USB3 hub", 1, 25.0)
    A("Compute & sensors", "Head RGB-D camera (Orbbec Gemini 2: 0.15–10 m, global-shutter IR, 98 g)", 1, 234.0, "store.orbbec.com (loop 14); Gemini 336 was $310")
    A("Compute & sensors", "Wrist camera (D12): global-shutter OV9281-class MIPI-CSI module + FFC through the wrist + CSI→USB (UVC) bridge on the forearm + slim 4-core high-flex USB (OD ≤ 2.2 mm)", 2, 55.0, "T01: USB versions $60–155; CSI module + bridge ESTIMATE")
    # --- sensor pack v2 (owner, loop 15: favour accuracy + dexterity over cost) ---
    enc = "MT6701 (D29 #6)" if K.JOINT_ENCODER == "mt6701" else "AS5048A"
    A("Sensor pack v2", f"Joint-side absolute encoders {enc} + diametric magnets on the shaft ends ({ns} arm belt joints + lift) → hand error 4.3 → "
      f"{__import__('rnd.accuracy', fromlist=['score']).score(d)[0]:.1f} mm (accuracy.md)", ns + 1, *K.ENCODER_P)
    if inline:
        A("Sensor pack v2", "Inline-roll joint encoder: off-axis ring magnet round the GIM8108-36 + encoder board + printed bracket (the motor fills the axis; "
          "the old 9-encoder line counted the roll as a belt joint)", 2, 20.0, "ESTIMATE (rnd/inline_study.py OFFAXIS_ENC, same in every variant)")
    A("Sensor pack v2", "Sensor nodes: RP2040 + CAN transceiver (shoulder + elbow node per arm, sw/config/can_map.yaml 0x30 / 0x31; one in the base)", 5, 10.0, "ESTIMATE")
    A("Sensor pack v2", "Fingertip tactile pads: magnetised elastomer + MLX90393 magnetometer (AnySkin/ReSkin principle), 2 per gripper", 4, 10.0, "ESTIMATE")
    A("Sensor pack v2", "Gripper ToF proximity (VL53L4CD) for pre-grasp alignment", 2, 8.0, "ESTIMATE")
    A("Sensor pack v2", "Base cliff / stair-edge ToF sensors (VL53L1X, downward)", 4, 8.0, "ESTIMATE")
    A("Compute & sensors", "2D lidar (RPLIDAR C1)", 1, 69.0, "RobotShop US $69 (2026-11); Waveshare $79.99")
    A("Compute & sensors", "IMU (BNO085) + optical-flow (PAA5100JE)", 1, 55.0, "components.yaml (flow)")
    A("Compute & sensors", "Mic array + speaker + chest/bumper LEDs (eyes are now drawn on the D11 screen)", 1, 36.0, "ESTIMATE")
    A("Compute & sensors", "D22 face screen: Waveshare ESP32-P4-WIFI6-Touch-LCD-7B (7\" 1024×600 cap. touch, CAN + RS485) + 24→5 V 2 A buck", 1, 46.99 + 6.0, "waveshare.com SKU 32510 (2026-11-27)")
    A("Drive & structure", f"D23 consumer covers: TPU 95A gaiters/bumper skins (1 kg) + fasteners; the rigid PETG covers ({COVERS_KG} kg, covers.printed()) are in the PETG bundles below", 1, 30.0,
      "ESTIMATE (TPU 95A ≈ $20–25 / kg)")
    A("Bearings", "6808-2RS (40×52×7), no-name 10-pack (check the 2RS variant): 2 per wheel + 2 per arm belt stage", 8 + 2 * ns, 2.02, "AliExpress MOCHU snapshot (loop 14); Zoro generic $4.71")
    A("Bearings", "61804-2RS (20×32×7): D16 wrist-roll spindles, 2 per hand (the lift's small hub pair went with the lift's D26 direct stage, W12)", 4, 0.92, "AliExpress MOCHU snapshot; Zoro $4.69")
    # D26: the 6801 pairs of the Oldham-driven Ø12 shafts go; one bearing per direct stage carries the stub's far end in wall B
    A("Bearings", f"6001-2RS (12×28×8): D26 direct stages, the drive stub's far end in wall B's bonded bushing ({ns} arm belt stages)", ns, *K.B6001)
    A("Bearings", "61901-2RS (12×24×6): the lift's direct stage, flush in wall B (the mast coupling is 2 mm behind it)", 1, *K.B61901)
    A("Bearings", "6805-2RS (25×37×7): head tilt passive pivot (A4)", 4, 1.15, "AliExpress snapshot; Zoro $5.69")
    A("Bearings", "U604ZZ grooved pulley bearings (A9 balancers: pulley A + tackle pulley)", 4, 1.5, "ESTIMATE")
    A("Bearings", "625ZZ (carriage, idlers)", 10, 0.25, "Walmart 40-pack")
    A("Bearings", "6708-2RS thin section (40×50×6), elbow belt cassettes (2 per arm)", 4, 3.0, "ESTIMATE")
    # --- self-docking charger (queue item 4, owner request; rnd/parts/dock.py, rnd/dock_sim.py) ---
    A("Docking charger", "6S Li-ion CC/CV charger 25.2 V, 4–5 A (≈ 2.5 h full charge)", 1, 35.0, "ESTIMATE")
    A("Docking charger", "Dock electronics: ESP32-C3 + MOSFET output switch + ID-pin handshake, fuse, DC jack, wiring", 1, 12.0, "ESTIMATE")
    A("Docking charger", "Contacts (Neon #2 F12): 2 spring leaf contacts + 1 pogo pin (dock); 3 brass pads 60 × 28 mm + reverse-polarity diode/fuse + ID-line 1 kΩ / 5.6 V TVS (robot)", 1, 14.0, "ESTIMATE")
    A("Docking charger", "Dock baseplate held down by the robot's wheels (no floor screws; dock_sim.md): 6 mm sheet with two wheel cradles + rubber feet + AprilTag print", 1, 12.0, "ESTIMATE")
    A("Drive & structure", "Torque hangers: 2 × M4×10 + 2 heat-set inserts each (wheels ×4 so far)", 4, 0.6, "ESTIMATE")
    A("Drive & structure", "Elbow belt cassette: 2 × M5×30 bolts + nylocs per arm; 4 × M3 heat-set inserts + screws for the bone cap", 2, 2.0, "ESTIMATE")
    # loop 40: 13 sockets (the wheels bolt on, D17/A11; the wrist roll lost its KX44, D16); D15 screwed sockets with 6 coil springs
    # wave 1 (D24): the 8 arm cartridges + 2 wrist DMs bolt in; W12 (lift on the D26 direct stage, a bolted cartridge) took the lift's KX74 too, and the
    # wheels bolt to the GIM output (D17, torque hangers below): only the head pan + tilt STS3215 cartridges (KX44, cartridge_s.py) keep a socket
    A("KX interfaces (2 sockets)", "G25 balls: 6 × Ø6 (KX44 × 2; the Ø10 set was the lift's KX74)", 1, 2.0, "Intech / Zoro listings, pro rata")
    A("KX interfaces (2 sockets)", "Hardened dowel pins: 50 Ø3×8 (KX44 pin tracks; the Ø5 × 14 pack was the KX74's; the Ø6 yaw-stop / ferrule dowels are below)", 1, 12.5,
      "Zoro listings (the Ø3 pack of the 2 packs priced at $25)")
    A("KX interfaces (2 sockets)", "D15 preload: 6 coil springs Ø4 × 8 per socket (assortment) + PTFE washer; 3 × M3 retainer screws + inserts", 2, 1.2, "ESTIMATE")
    A("KX interfaces (2 sockets)", "Torque pins Ø4 + magnet index (head pan + tilt cartridges)", 2, 0.4)
    A("Drive & structure", "Dowels Ø6 × 10–12: yaw hard-stop pins + KX-V ferrule dowels (3 per arm)", 1, 6.0, "ESTIMATE")
    # D26 (owner cost-down, wave 11): no Oldham couplings. Each GIM6010 stage (6 arm belt stages + the lift = 7) drives its keyed 20T
    # from a turned C45 stub bolted to the motor output (belt_stage.small_stage_v4); the 6061 motor hubs, D50 sets, spare discs and
    # the keyed Ø12 rod share go
    # Astra #3 m7 (W13c): stub v5 = one C45+QT integral pinion shaft per stage, the 20T hobbed on it (no keyed aluminium pulley, no key)
    A("Drive & structure", f"D26 integral pinion shaft (stub v5) ×{ns + 1} ({ns} arm belt stages + the lift): turned + hobbed C45+QT, Ø42 × 4 flange bolted to the "
      "GIM output (8 × Ø3.4 + c'bore), Ø16/10 neck, 20T HTD 5M teeth on the shaft (motor-side flange), Ø15 collar, Ø12 k5 seat for the wall-B bearing "
      "(belt_stage.small_stage_v4; replaces the v4 stub + keyed aluminium 20T + key)", ns + 1, *K.PINION)
    A("Drive & structure", f"Stub screws: DIN 7984 M3 × 6 8.8 (8 per stub, {8 * (ns + 1)}), 100-pack; Loctite 243 from the fasteners line", 1, *K.M3X6_PACK)
    A("Drive & structure", "D26 pitch idler (×2, one per arm): 2 × 625-2RS on an M5 × 40 axle (cut to 38) + captive nut + spacers, M4 jack screw + nut "
      "(the roller is printed; elbow / yaw run at fixed centres)", 2, 2 * K.B625[0] + K.IDLER_HW[0], K.B625[1] + "; " + K.IDLER_HW[1])
    A("Drive & structure", "D26 pitch exit-slot strap (×2): 3 mm 6061-T6 70 × 14 across the cartridge's exit slot + 4 × M4 × 12 + 2 heat-set inserts "
      "(the key is printed)", 2, K.STRAP[0] + K.STRAP_HW[0], K.STRAP[1] + "; " + K.STRAP_HW[1])
    A("Drive & structure", "D26 tension ring (Astra #3 m6) at the elbow and yaw of each arm: turned 6061-T6, bore eccentric 0.30 mm, under the cartridge's "
      "centring lip, set by span frequency before the 6001 bond (belt_stage.TENSION_RING; belts.md: 0–165 N → 134–220 N preload)",
      2 * len(TENSION_RING_SITES), *K.TENSION_RING)
    A("Drive & structure", "D19 elbow shaft core (W8b, Neon #2 F7): 6061-T6 tube Ø30 × 1.5, 2 × 139.5 mm, bonded (takes ≈ 85 % of the bending)", 1, *K.ELBOW_TUBE)
    A("Drive & structure", "Structural epoxy for the elbow core tubes (and the CF bonds): Loctite EA 9466 / 3M DP420 class, 50 ml + nozzles", 1, *K.EPOXY)
    A("Drive & structure", "D24/D19 heat-set inserts: 2 × 100 M4 short (≈ 51 per arm: flanges 6 each, seat caps, yaw flange + plate, outer wall, pitch hub) + 100 M3 (≈ 45 per arm: cap rims, forearm rim, palm, bone cap)", 1, 2 * 15.93 + 12.0,
      "Newark M4 brass $15.93 / 100; M3 ESTIMATE")
    A("Drive & structure", f"D24 v2 flange ears (W8b, Neon #2 F2: 4 ears per stage × {ns + 1}): 24 × ISO 4762 M4 × 12 + 4 × M4 × 16 (pitch rear pair, from the pulley side), "
      "28 × ISO 4032 M4 nuts, 28 × ISO 7089 washers (the 70 countersunk M4 × 12 of the D24 v1 flange go)", 1, 9.0, "ESTIMATE")
    # C1 joints (Fable C1 + W8c / Neon #2 F3): the hardware lines are the M1 kit's (Step 2 common), per arm
    for g, n, q, u, w, src in K.step2_common():
        if g.startswith("Joints"):
            A("Drive & structure", f"C1 joint, both arms (M1 kit line × 2): {n}", 2 * q, u, src)
    A("Drive & structure", "Wheel hubs bolted to the GIM output (D17/A11: no KX on the wheels): 4 × M4 per wheel", 4, 0.8, "ESTIMATE")
    A("Drive & structure", "Chassis ring (A11): 3 mm 6061 laser-cut ring tying the 4 deck quarters", 1, 40.0, "ESTIMATE (SendCutSend class)")
    # W8d (Neon #2 F4, A10 v2): D18b lockable casters + removable rear ballast (rnd/parts/base.py)
    # D26 (wave 11): the 2" ball transfers (≈ $45 each, ESTIMATE; 2" units are industrial quote items) → printed PTFE skid feet (base.caster_unit)
    A("Drive & structure", "D18b v2 skid feet: Ø19 PTFE screw-on glides (one per foot + spares) on the printed 44° foot at r 205 on the 4 diagonals (22 mm float; "
      "a 2–22 mm step meets the flank: climb ≈ 1.3 × the spring force vs 2.2 × for the 2\" ball)", 1, *K.PTFE_GLIDES)
    A("Drive & structure", "D18b v2 skid feet: PTFE tape on the 4 flanks (the feet themselves print in PETG, ≈ 25 g each, from the bundle's spare)", 1, *K.PTFE_TAPE)
    A("Drive & structure", "D18b caster mechanics: 2 × Ø8 h6 guide rods (≈ 90 mm), spring-applied one-way rod lock plate (caulk-gun type), 5 N spring, steel top plate", 4, 8.0,
      "ESTIMATE (8 mm linear rod ≈ $6 / 300 mm; plates on the SendCutSend order)")
    if d.base.caster_release == "cable":                        # D29 #9 (rnd/tip.py cable_release: JF-1039 0.84×, JF-1264B 1.85× / 1.06×)
        A("Drive & structure", "D18b lock release (D29 #9): ONE central 24 V pull solenoid, 55 N, pulling the 4 spring-applied lock plates through Ø1 cables "
          "(e-stop / power loss = all locked)", 1, *K.SOLENOID_55N)
        A("Drive & structure", "D18b release cables: 4 × Ø1 steel cable in PTFE liners + crimp stops + printed equaliser", 1, *K.CABLE_KIT)
        A("Drive & structure", "D18b solenoid driver: 1 × logic-level MOSFET + flyback diode + JST, on the power board (SW-T2 caster-lock output)", 1, 4.0, "ESTIMATE")
    else:
        A("Drive & structure", "D18b lock release: 24 V pull solenoid (spring-applied lock: e-stop / power loss = locked)", 4, *K.SOLENOID)
        A("Drive & structure", "D18b solenoid drivers: 4 × logic-level MOSFET + flyback diode + JST, on the power board (SW-T2 caster-lock output)", 1, 6.0, "ESTIMATE")
    A("Drive & structure", "A10 v2 rear ballast (D26): 2 × 2.45 kg removable blocks, each one saw-cut 92 mm drop of 1-3/4 × 3\" A36 flat bar (44.5 × 76.2; was 7 laser-cut "
      "1/4\" plates + bolts, $100), ends deburred", 1, *K.BALLAST_BAR)
    A("Drive & structure", "Ballast pull loops: 2 × 25 mm polyester webbing under the blocks (lift-out, R3)", 1, 2.0, "ESTIMATE")
    A("Drive & structure", "Clock-spring ribbon stock (A3, Neon #2 F10): 16 AWG high-flex ETFE/FEP pairs (was 20 AWG silicone), 26 AWG twisted pairs, shielded slim USB, PET/Kapton carrier + PTFE tape liners", 1, 40.0, "ESTIMATE")
    A("Drive & structure", "HTD 5M 15 mm closed belts (" + ", ".join(f"{j} {l} mm" for j, l in belts) + ") × 2 arms", 2 * len(belts), 9.0, "Bestorq 400-5M-15 $7.67 at MROSupply (loop 14); Gates ≈ $20–28")
    A("Drive & structure", "HTD 5M 15 mm open belt for the lift (2.4 m at $8.32/m) + 2 clamps", 1, 25.0, "Acorn listing")
    if inline:                                                  # D24 inline roll (rnd/parts/roll.py INLINE section), per arm
        A("Bearings", "61806-2RS (30×42×7): inline roll rear support on the roll link's stub", 2, 3.0, "ESTIMATE (m1_kit Hold B)")
        A("Drive & structure", "Inline roll output adapter: 7075-T6 Ø76 × 7, Ø30 pilot, 3 × Ø4 H7 reamed + 3 × M4 csk, 8 × M5 tapped on PCD 64 (CNC, qty 2)", 2, 30.0,
          "ESTIMATE: JLCCNC / PCBWay-class instant quote for a 3-axis Ø76 × 7 part ≈ $20–25 each at qty 2 + shipping share; a printed adapter fails (3 × M4 into PETG ≈ 85 N·m < 103 N·m stall, roll.py)")
        A("Drive & structure", "Inline roll hardware: Ø80/Ø52 × 2 steel clamp ring (laser-cut, rides on the chassis-ring order), 8 × M5×16 DIN 7984 10.9, "
          "3 × M4×10 ISO 10642, 3 × Ø4 m6 dowels, end washer + M5 (W8c: the 8 × M4×18 + washers + DIN 988 shims are replaced by the rear-mount lines below)", 2, 9.0,
          "ESTIMATE (ring ≈ $5 SendCutSend class; screws/dowels ≈ $4)")
        for g, n, q, u, w, src in K.HOLD_B["yes"]:               # W8c rear mount + clevis (Neon #2 F3/F6), per arm; the Loctite is one buy
            if "W8c" in g:
                once = n.startswith(("Loctite", "E235+C", "Sleeve facing"))       # one buy for the robot (D26: the tube sticks + jig serve both arms)
                A("Drive & structure", f"Inline roll (W8c{'' if once else ', both arms'}): {n}", q if once else 2 * q, u, src)
    A("Drive & structure", "Al square tube 60×60×2, 1.1 m (mast), black anodised", 1, 45.0, "no US listing found: ESTIMATE")
    A("Drive & structure", "Carbon tubes Ø40×38 (0.5 m) + Ø30×28 (1 m), roll-wrapped", 1, 70.0, "eBay / Carbonwebshop listings")
    A("Drive & structure", K.DETACH_CLAMP_P[1] + " (arm detach; flange OD 90.9 mm confirmed; D29 #10)", 2, K.DETACH_CLAMP_P[0], K.DETACH_CLAMP_P[2])
    A("Drive & structure", "Lift lock (A15): bolted steel rack (module 1, 10 × 10, 500 mm) + hardened steel pawl + 12 V solenoid + springs; printed belt guards", 1, 35.0, "ESTIMATE")
    A("Drive & structure", "Balancers (A9): ISO 10243 green die spring Ø25/Ø12.5 × 115, 18.7 N/mm (Berger Tools V25-115 / Accu TJL25-115) + 2 mm UHMWPE cord", 2, 12.0, "ESTIMATE")
    A("Drive & structure", "Fasteners (other), heat-set inserts (base, torso, head), epoxy, grease", 1, 60.0)
    A("Drive & structure", f"W14b R17 roll front collars (covers.roll_front_collar, one per arm): {W14B['screws']} × ISO 4762 M4 × 25 (4 per collar); the "
      "collars' PETG is in the PETG line", W14B["screws"], *W14B["screw_p"])
    cov = pp.get("covers_kg", 0.0)                              # W10: the print plan now slices the D23 / R17 covers itself
    w14b = abs(cov - W14B["pp_covers_before"]) < 1e-3           # the print plan predates W14b: add its net PETG until it is re-sliced
    kg = pp["kg"] + (0.0 if cov else COVERS_KG) + (W14B["kg"] if w14b else 0.0)
    n10 = math.ceil(kg * 1.2 / 10)                              # loop 47: 10 kg bundles instead of $20/kg single spools
    what = f"PETG {pp['kg']:.1f} kg (print_plan.md, the {cov:.1f} kg of D23 / R17 covers included)" if cov else f"PETG {pp['kg']:.1f} kg (print_plan.md) + {COVERS_KG} kg D23 covers"
    if w14b:
        what += (f" + {W14B['kg']:.2f} kg W14b pinch parts (roll front collars +{W14B['cm3'][0]} cm³, shoulder shrouds +{W14B['cm3'][1]}, roll links "
                 f"−{-W14B['cm3'][2]}; print-plan basis, the plan predates them)")
    A("Printed parts", f"{what} + 20 % spare = {kg * 1.2:.1f} kg: {n10} × Elegoo PETG Pro 10 kg bundle", n10, 109.99,
      "us.elegoo.com PETG Pro 10 kg $109.99 (2026-11; list $140)")
    A("Printed parts", "ASA 2 kg (A6: motor cans, KX hubs/collars and D13 hubs near motors)", 1, 40.0)
    return I


# loop 47 cost pass: the same parts at the best verified price (motor choices unchanged); wave-2 cost pass (2026-11-27) below
BEST_PRICE = {"GIM6010-8": (K.GIM6010[0], "D29 #2: " + K.GIM6010[1] + f" (+${K.GIM6010_FALLBACK[0] - K.GIM6010[0]:.0f} a unit); "
                                           "SteadyWin store $93.88 with GDS68"),
              "GIM8108-36": (240.0, "with a driver: SteadyWin store $239.80 (GDM810 = SDC303, 24 V), aifitlab $240 (GDK810); bare motor $190 "
                                    "(aifitlab, backordered), so the yaml mid $171 had no driver (2026-11-27 search snapshots)"),
              "DM-J4310-2EC_V1.1": (125.0, "AIFit Lab $125 (24 V V1.1/V3, US seller; backordered 2026-11-27): the full-robot buy "
                                           "comes after M1; M1 keeps Robotics Center $150 in stock")}
COVERS_KG = 1.56        # D23 rigid PETG covers, both arms: 1,126 cm³ (rnd/parts/covers.py printed(), arm pieces ×2) × 1.27 g/cm³ = 1.4 kg, + the W14b
                        # roll front collars 128 cm³ (both arms) × 1.27 = 0.16 kg (solid basis; the PETG line reads the print plan's sliced mass)
# W14b R17 pinch parts (rnd/parts/covers.py roll_front_collar + the pitch port shroud, rnd/parts/roll.py link frame), robot (both arms):
# cm³ (collar, shroud, roll link); net printed mass on the print-plan basis (≈ +80 g collars, net ≈ +70 g); print_plan.json's covers_kg
# before W14b (the PETG line adds kg only while the plan still shows that value, so a re-sliced plan is not counted twice)
W14B = dict(cm3=(128, 20, -38), kg=0.070, pp_covers_before=1.9822, screws=8,
            screw_p=(0.30, "ESTIMATE: ISO 4762 M4 × 25 A2 / 8.8, ≈ $0.30 each in a 25–50 pack"))

CONTINGENCY = 0.15      # A16 (review finding 18): price drift, shipping, tax
WAVE1_BEFORE = 4898.0   # landed total after the loop-47 cost pass (bom.json before wave 1)
WAVE1_AFTER = 5013.0    # after wave 1 (RND_LOG, buy-list track)
WAVE2_BEFORE = 5199.0   # after the owner's inline-roll OK, before the wave-2 cost pass
WAVE2_AFTER = 5308.0    # after the wave-2 cost pass (before the Fable-review fixes)
NEON2_BEFORE = 5485.0   # bom.json before the Neon review #2 electrical / harness / dock / keyed-drive fixes (2026-11-27)
W8_BEFORE = 5845.0      # bom.json before the W10 consolidation of the W8b / W8c / W8d hardware (2026-11-27)
D26_BEFORE = 7949.0     # bom.json after W10, before the D26 cost-down (wave 11)
# D26 sleeves / casters / ballast: (item, before $ of lines, after $ of lines, the check before → after); lines without contingency
D26_ROWS = [
    ("Shaft couplings out (D26 direct stages): 7 D50 Oldham sets + 6061 motor hubs + 2 spare discs + the keyed-rod share + 14 × 6801 → 7 turned C45 stubs "
     "+ 6 × 6001 + 1 × 61901, 2 pitch idlers (2 × 625 + axle + jack screw), 2 exit-slot straps; one M3 × 6 100-pack either way",
     7 * (K.OLDHAM[0] + K.OLDHAM_MOTOR_HUB[0]) + 2 * 36.45 + 2 * K.KEYED_SHAFT[0] + 14 * K.B6801[0],
     7 * K.STUB[0] + 6 * K.B6001[0] + K.B61901[0] + 2 * (2 * K.B625[0] + K.IDLER_HW[0]) + 2 * (K.STRAP[0] + K.STRAP_HW[0]),
     "belts.md / joints.md D26 tables: 6001 C0 vs the GIM6010 peak, stub 8 × M3, pitch idler preload; lift 61901 2.0× (714 N); spacing 0 hard; "
     "assembly_seq R14 (c)/(d) incl. the lift and the D20 pitch swap"),
    ("28 clevis + rear crush sleeves: turned 42CrMo4 → cut and faced E235+C tube (8 × 1.2, 6 × 0.8) + facing jig",
     28 * K.SLEEVE[0], 2 * K.SLEEVE_TUBE[0] + K.FACING_JIG[0],
     "joints: rear-mount friction 1.59 → 1.60× (softer sleeve, less embedding loss); new sleeve-yield rows: rear 139 MPa (1.69×), clevis 100 MPa (2.34×) at 235 MPa; pitch / yaw unchanged"),
    ("D18b casters: 2\" ball transfer → printed skid foot + PTFE glide + PTFE tape (rods, lock, solenoids unchanged)",
     4 * K.BALL2[0], K.PTFE_GLIDES[0] + K.PTFE_TAPE[0],
     "A10 16/16 ≥ 30 mm, worst 34 → 35 mm; R4 threshold 1.71 → 1.70× (+0.12 kg); float climb 2.2 → 1.3 × the spring force"),
    ("Ballast: 14 laser-cut 1/4\" plates + bolts → two saw-cut 1-3/4 × 3\" bar drops + pull loops",
     14 * K.BALLAST_PLATE[0] + 2.0, K.BALLAST_BAR[0] + 2.0,
     "same 2 × 2.45 kg (FROM CAD), centroid 5 mm further back and 2.6 mm higher; tip within the caster row"),
]
# D26: what is left of the gap to $5,500 after the couplings went. (match on the item name, credible alternative, saving
# in $ of lines before contingency, as a function of the line's qty); the top lines by landed cost come from items().
GAP_ALTS = [
    (lambda n: n == "GIM6010-8", "already at Robotics Center's $70 (D29 #2, backordered; $90 fallback); the 4 wheel units could only "
     "go cheaper with a base-drive redesign (not costed)", lambda q: 0.0),
    (lambda n: n == "GIM8108-36", "belt roll fallback (PIP_ROLL_DRIVE=belt: a GIM6010-8 on 18T:80T, CAD and checks in the tree); the owner chose the inline "
     "roll for its hand-feel and slimmer shoulder", lambda q: HOLDS["inline"] / (1 + CONTINGENCY)),
    (lambda n: n.startswith("DM-J4310"), "none sourced cheaper in the US: Foxtech $116–120 ships from China (duty eats the gap); $125 is AIFit Lab, "
     "backordered", lambda q: 0.0),
    (lambda n: n.startswith("Head RGB-D camera"), "none: the OAK-D Lite is now $269 (Luxonis list since 2026-07-09, 2026-11-27 search); Gemini 2 is the "
     "low listing for a global-shutter RGB-D", lambda q: 0.0),
    (lambda n: n.startswith("PETG"), "none: the plan's kg + 20 % spare needs 2 bundles; one bundle would need ≤ 8.3 kg printed", lambda q: 0.0),
    (lambda n: "rush sleeves, turned" in n, "the 16 pitch sleeves from 1/4\" × .035\" 4130 tube once shoulder.py's sleeve hole goes Ø7.1 → Ø6.45 "
     "(rnd/joints.py SLEEVE_STOCK; pitch stall row 1.18 → 1.08×); the yaw's Ø4.6 stays turned", lambda q: 16 * (K.SLEEVE[0] - K.SLEEVE_ALT[0])),
]


def gap_table(I, n=5):
    """Top n BOM lines by landed cost [(item, qty, landed $, alternative, landed saving)], then the next line with a costed
    alternative if it is not among them."""
    c = 1 + CONTINGENCY
    rows = sorted(I, key=lambda r: -r[2] * r[3])
    alt = lambda nm: next(((a, f) for m, a, f in GAP_ALTS if m(nm)), ("none listed", lambda q: 0.0))
    out = [(nm, q, q * u * c, alt(nm)[0], alt(nm)[1](q) * c) for _, nm, q, u, _ in rows[:n]]
    for _, nm, q, u, _ in rows[n:]:
        a, f = alt(nm)
        if f(q) > 0:
            out.append((nm, q, q * u * c, a, f(q) * c))
            break
    return out


OLD_LINES = dict(oldham=6 * 20.0, casters=4 * 4.0, inline_hw=2 * 2.0, flange_screws=10.0)   # the $5,845 list's lines that W10 replaced
HOLDS = {}


def drivers(d=BASELINE):
    """Today's added cost (W8b / W8c / W8d + the C1 hardware the BOM still lacked), landed, ranked, each with the cheapest
    credible alternative and its landed saving (prices: rnd/m1_kit.py constants)."""
    c = 1 + CONTINGENCY
    I = items(d)
    g = lambda f: sum(q * u for _, n, q, u, _ in I if f(n))
    is_sl = lambda n: "rush sleeves" in n or "hard-stop bushes" in n or "precision tube" in n or "Sleeve facing" in n
    rows = [
        ("D26 direct drive (was the D50 Oldham + 6061 motor hub, W10): 7 C45+QT integral pinion shafts (v5, the 20T on the shaft) + M3 × 6, the pitch idlers and exit-slot straps",
         g(lambda n: n.startswith(("D26 integral pinion", "Stub screws", "D26 pitch"))) - OLD_LINES["oldham"],
         "none cheaper sourced: the stub is the one custom metal part per stage (ESTIMATE until the STEP is quoted)", 0.0),
        ("Crush sleeves + stop bushes (W8c): 28 turned 42CrMo4 (pitch, yaw) + 4 stop bushes; since D26 the 28 clevis + rear sleeves are cut from stock tube",
         g(is_sl), "the 16 pitch sleeves from 1/4\" × .035\" 4130 tube (F_ty 517) once shoulder.py's sleeve hole goes Ø7.1 → Ø6.45 (located by the hole; "
         "the pitch stall row 1.18 → 1.08×, rnd/joints.py SLEEVE_STOCK); the yaw's Ø4.6 has no stock tube that clears its stall stress", 16 * (K.SLEEVE[0] - K.SLEEVE_ALT[0])),
        ("D18b lockable casters (W8d; D26: PTFE skid feet instead of 2\" ball transfers): glides + tape, rods + lock, 24 V solenoids, drivers", g(lambda n: n.startswith("D18b")) - OLD_LINES["casters"],
         "D29 #9 done where PIP_CASTER_RELEASE=cable: one 55 N solenoid pulling 4 cables (the 25 N JF-1039 cannot: tip.md); nothing cheaper sourced", 0.0),
        ("C1 joint hardware both arms (tie-rods, 6061 yaw flange, 7075 crank): in the M1 kit, never added to the BOM", g(lambda n: n.startswith("C1 joint") and "sleeves" not in n),
         "none: the printed joints are what failed (Fable C1)", 0.0),
        ("Rear ballast 2 × 2.45 kg (W8d; D26: saw-cut 1-3/4 × 3\" flat bar instead of laser-cut plate stacks)", g(lambda n: "allast" in n), "none (a local yard drop is the floor)", 0.0),
        ("Inline roll rear mount: 7075 plate rings, M4 × 25 screws, DIN 471, Loctite 660 + 638 (W8c), less the M4 × 18 + shims",
         g(lambda n: n.startswith("Inline roll (W8c") and not is_sl(n)) - OLD_LINES["inline_hw"],
         "Loctite 660 at the $28.99 retail pack already; none cheaper sourced", 0.0),
        ("Elbow 6061 core tubes + structural epoxy (W8b)", g(lambda n: "elbow shaft core" in n or n.startswith("Structural epoxy")), "none (Neon #2 F7 fix)", 0.0),
        ("Lift direct stage + 61901 (lift v2 is a bolted stage; D26 direct, v5 pinion)", K.PINION[0] + K.B61901[0], "none", 0.0),
    ]
    return sorted([(w, a * c, alt, s * c) for w, a, alt, s in rows], key=lambda r: -r[1])


def _sd(x):
    return f"{'+' if x >= 0 else '−'}${abs(x):,.0f}"


def holds():
    """Robot-wide deltas (with contingency) of the M1 kit's two holds (rnd/m1_kit.py prices), both arms."""
    from . import m1_kit as k
    c = 1 + CONTINGENCY
    a = k.line_sum
    return dict(d20=2 * (a(k.HOLD_A["yes"]) - a(k.HOLD_A["no"])) * c, d20_hi=2 * k.HOLD_A["hi"]() * c,
                inline=2 * (a(k.HOLD_B["yes"]) - a(k.HOLD_B["no"])) * c, inline_hi=2 * k.HOLD_B["hi"]() * c)


# --- Waves 12–13 refresh (2026-11-28): line changes against the W12 total (bom.json $6,609 before this pass) -------------------------
W12_TOTAL = 6609.0
# (line, before $ of lines, after $ of lines, what changed and the source); no contingency
REFRESH_ROWS = [
    ("D26 drive: 7 × (turned C45 stub + keyed aluminium 20T + key) → 7 × C45+QT integral pinion shaft, 20T hobbed (stub v5, Astra #3 m7: the keyed "
     "Ø12 was 0.68× (D20) / 1.19× (GIM6010) in rotating-bending fatigue → 2.63× / 3.13×)",
     7 * K.STUB[0] + 7 * 19.5 - 2 * K.KEYED_SHAFT[0], 7 * K.PINION[0], "belts.md: ≈ $35 each in 10s (ESTIMATE until the STEP is quoted)"),
    ("D26 tension rings (Astra #3 m6), elbow + yaw × 2 arms, turned 6061-T6 e 0.30 mm (the brief said steel: belt_stage.TENSION_RING is 6061-T6)",
     0.0, 2 * len(TENSION_RING_SITES) * K.TENSION_RING[0], "fixed-centre preload 0–165 N → 134–220 N (belts.md); $8 each ESTIMATE (m1_kit TENSION_RING)"),
    ("C1 clamp rings 6061 / S235 → C45+QT (Astra #3 m9), both arms; yaw screws M3 × 45 → M3 × 40", 2 * 12.0, 2 * (4.0 + 2 * K.C45_RING[0]),
     "joints.md contact faces: 0.64–0.88× → 1.34–2.52×"),
    ("C1 yaw flange 6061-T6 → 7075-T6 (Astra #3 m9), both arms", 2 * 25.0, 2 * K.YAW_FLANGE_7075[0], "sleeve foot on the spigot 0.75× → 1.43× (joints.md)"),
    ("Safety-board netlist v3 (Astra #3 critical 3): 100 V coil driver, 50 µs min-on, AUX / K1 discrepancy monitor, 12 V UVLO, 2 arm-branch current trips",
     0.0, K.BOARD_V3[0] - K.ACS724[0] + 1.0 + 2 * K.ACS724[0], "electrical.md netlist U16–U19 / Q4; safety_board_sim.md design changes 0–5"),
    ("Encoders: 9 × AS5048A ('8 arm belt joints + lift') → 7 × AS5048A + 2 off-axis ring encoders on the inline roll", 9 * 12.0, 7 * 12.0 + 2 * 20.0,
     "inline_study.py OFFAXIS_ENC (the roll is not a belt joint since D24)"),
    ("Lift small-hub 6804 pair (gone with the W12 direct stage): 6 → 4 × 61804", 6 * 0.92, 4 * 0.92, "rnd/parts/upper.py lift v2"),
    ("KX sockets 3 → 2 (the lift is a bolted cartridge since W12): Ø10 balls, the Ø5 dowel pack, one preload set, 5 torque pins out",
     3.0 + 25.0 + 3 * 1.2 + 7 * 0.4, 2.0 + 12.5 + 2 * 1.2 + 2 * 0.4, "rnd/parts/upper.py, base.py (D17), cartridge_s.py (head KX44)"),
    ("Aluminium finned rear caps on the 6 arm belt joints + NTCs (unchanged; now traced to thermal.md §3, the mount the thermal rows assume)",
     6 * K.AL_CAP[0] + 6.0, 6 * K.AL_CAP[0] + 6.0, "m1_kit AL_CAP ESTIMATE"),
]


def options(d=BASELINE):
    """Priced options, never in the total: (item, qty, unit $, source, note)."""
    return [(n, 2, u, src, w) for _, n, q, u, w, src in K.OPTIONS]


# --- cost-down view (docs/rnd/cost_down.md) ---------------------------------------------------------------------------------------
SUBSYS = ["Arms ×2: actuators", "Arms ×2: drive, joints, bones", "Base: wheels, tip, ballast", "Lift + mast", "Head + face",
          "Power + safety electronics", "Compute, sensing, comms", "Printing + covers + misc. fasteners", "Dock"]


def _split(n, q, u, d=BASELINE):
    """[(subsystem, $ of lines)] for one BOM line."""
    v = q * u
    if n == "GIM6010-8":
        arm = 2 * sum(1 for a, _ in d.arm.joints.values() if a == n)
        lift = 1 if d.upper.lift_motor == n else 0
        return [(SUBSYS[0], arm * u), (SUBSYS[2], 4 * u if d.base.wheel_motor == n else 0.0), (SUBSYS[3], lift * u)]
    if n.startswith("STS3215"):
        return [(SUBSYS[0], (q - 2) * u), (SUBSYS[4], 2 * u)]
    if n.startswith(("GIM8108", "DM-J")):
        return [(SUBSYS[0], v)]
    if n.startswith("6808-2RS"):
        return [(SUBSYS[2], 8 * u), (SUBSYS[1], (q - 8) * u)]
    if n.startswith(("D26 integral pinion", "D24 v2 flange ears")):        # one per GIM6010 belt stage: the arms' and the lift's
        return [(SUBSYS[1], v * (q - 1) / q if q > 1 else v * 6 / 7), (SUBSYS[3], v / q if q > 1 else v / 7)]
    lo = n.lower()
    rules = [
        (SUBSYS[6], ("usb-can", "usb-ttl", "active cooler", "usb3 hub", "rgb-d", "wrist camera", "encoder", "sensor nodes", "tactile", "tof", "lidar", "imu (",
                     "raspberry pi")),
        (SUBSYS[5], ("li-ion pack", "bms", "battery box", "fuses", "e-stop", "regen clamp", "safety-board", "branch current", "dc-dc", "connectors", "elbow-node power",
                     "base channel")),
        (SUBSYS[2], ("omni wheel", "wheel hubs", "torque hangers", "chassis ring", "d18b", "ballast")),
        (SUBSYS[3], ("lift", "61901", "mast", "open belt")),
        (SUBSYS[4], ("6805", "g25 balls", "dowel pins: 50", "d15 preload", "torque pins", "face screen", "mic array")),
        (SUBSYS[7], ("petg", "asa ", "consumer covers", "fasteners (other)")),
    ]
    for sub, keys in rules:
        if any(k in lo for k in keys):
            return [(sub, v)]
    return [(SUBSYS[1], v)]


def subsystems(I, d=BASELINE):
    """{subsystem: $ of lines} plus the dock (its own category)."""
    out = {k: 0.0 for k in SUBSYS}
    for c, n, q, u, _ in I:
        if c.startswith("Contingency"):
            continue
        if c == "Docking charger":
            out[SUBSYS[8]] += q * u
            continue
        for k, v in _split(n, q, u, d):
            out[k] += v
    return out


def savings(I):
    """Top cost-down candidates that keep the R2 / R17 / A10 / thermal gates, plus the queued belt-roll idea (flagged). Each: dict
    with lines $ saved (before contingency), what changes, risk, checks, owner, gates touched."""
    g = lambda f: sum(q * u for _, n, q, u, _ in I if f(n))
    metal = lambda n: n.startswith(("D26 integral pinion", "D26 tension ring", "D26 pitch exit-slot", "Chassis ring")) or (
        n.startswith("C1 joint") and "sleeves" not in n) or "output adapter" in n or "Rear plate ring" in n or "hard-stop bushes" in n
    batch = 0.15 * g(metal)
    n_gim = sum(q for _, n, q, _, _ in I if n == "GIM6010-8")
    return [
        dict(t="Belt shoulder roll instead of the inline GIM8108-36 (queued idea; `PIP_ROLL_DRIVE=belt`, CAD + checks in the tree)",
             s=HOLDS["inline"] / (1 + CONTINGENCY),
             change="2 × GIM8108-36 + inline adapter / rear mount → 2 × GIM6010-8 on 18T:80T belt stages (pinion v5, 475 mm belt, 2 × 61808, Al cap) — reverses D24",
             risk="**Not gate-neutral: R16 across-body 52 % on the belt roll (target 54 %) vs 60 % inline (roll_inline.md; 71 % in today's dexterity.md) and the dome grows 121 × 96 → 148 × 211 mm; hand-feel and "
                  "roll speed (48 → 27 rpm) were the owner's reasons for D24.** The roll latch option does not apply to the belt roll",
             checks="R17 rnd.pinch and A10 rnd.tip on PIP_ROLL_DRIVE=belt (not re-run since W13), R2d basket (not run on the belt roll), thermal (roll GIM6010 "
                    "in a sleeve: needs its Al cap), roll_inline.md all rows", own="**owner** (D24 reversal); roll track (rnd/parts/roll.py) re-runs",
             gates="R16 ❌, R17 / A10 / R2d to re-run", flag=True),
        dict(t=f"GIM6010-8 at the $70 listing ({n_gim} units)", s=n_gim * (90.0 - 70.0),
             change="same motor, same SDC104 24 V driver, bought at Robotics Center's $70 list (backordered 2026-11-27) instead of aifitlab's $90",
             risk="backorder / lead time; a different reseller may ship the 48 V or GDS68 driver variant (seller question 6)",
             checks="seller question 6 answered in writing (24 V SDC104, drawing); arrival check of the label + firmware (M1 CHECKS)",
             own="cost/BOM (sourcing); owner buys", gates="none"),
        dict(t="One batch quote for the custom metal (pinions, tension rings, C1 rings / flanges / cranks, roll adapters, rear plates, stop bushes, straps, chassis ring)",
             s=batch, change="quote all STEP files as one order at the robot quantity + M1 spares (qty 2–10 per part) instead of per-line instant quotes; "
                             "same drawings, same materials and certificates",
             risk="the −15 % is an ESTIMATE (no quote yet); one supplier's lead time gates everything; first-article inspection on the pinion teeth",
             checks="none on the design; incoming inspection (pinion profile vs a stock HTD 5M belt, C45+QT / 7075-T6 certificates, PCD on the measured GIM output)",
             own="cost/BOM", gates="none"),
        dict(t="Pitch crush sleeves from 1/4\" × .035\" 4130 tube (queued idea; 16 per robot)", s=16 * (K.SLEEVE[0] - K.SLEEVE_ALT[0]),
             change="shoulder.py sleeve hole Ø7.1 → Ø6.45 so the stock tube is located by the hole; joints.py SLEEVE_STOCK pitch → 4130 (F_ty 517); cut + faced in the existing jig",
             risk="R2 margin shrinks but stays a pass: pitch stall row (thread strip) 1.18 → 1.08× (rnd/joints.py); butt overlap of the two sleeves at the hub ↔ tube face",
             checks="rnd.joints (stall + bump rows ≥ 1.0 / 1.5), rnd.assembly_seq and rnd.spacing (the hole change), rnd.basket rows; a sleeve coupon in COUPONS.md",
             own="shoulder CAD track (rnd/parts/shoulder.py) + joints.py", gates="R2 (passes at 1.08×)"),
        dict(t="Regen-clamp NH-50 0.56 Ω at the distributor price (3 on the robot, 3 on the M1 board)", s=3 * (30.0 - 15.0),
             change="no design change: the line's $30 each is an ESTIMATE; the same Vishay NH-50 series lists at $15.01 (IBS, another value)",
             risk="the 0.56 Ω value may carry a premium or a minimum order; do not substitute the wirewound RH-50 (Neon #2 F1)",
             checks="part number starts NH050 (M1 CHECKS); pulse rating confirmed with Vishay (electrical.md)", own="electrical track + cost/BOM", gates="none"),
        dict(t="Stock aluminium heatsinks for the 6 finned rear caps", s=6 * (K.AL_CAP_MACHINED[0] - 6.0),
             change="a stock extruded round heatsink (Ø86–96 LED-downlight class, ≈ $6 ESTIMATE) faced flat and drilled to the cap pattern, on a thermal pad, "
                    "instead of a CNC'd finned 6061 disc",
             risk="fin area and flatness unknown until a sample is measured; the capped thermal rows need a cap contact resistance of ≤ 10.5 K/W (B0 pitch, worst 1 kg hold) and ≤ 40 K/W (yaw / elbow holds); capless the task day is 102 °C (limit 100)",
             checks="rnd.thermal with the stock part's h_o·A_fins and contact resistance (every §3 row ≤ 100 °C in the band); Step-1 lever test with the cap fitted",
             own="cartridge + thermal track (cartridge_m.py, thermal.py)", gates="thermal (must re-pass)"),
        dict(t="MT6701 magnetic encoders for the 7 shaft-end joints", s=7 * (12.0 - 5.0),
             change="AS5048A boards ($12 ESTIMATE) → MT6701 14-bit boards (≈ $5 ESTIMATE), same diametric magnet; SSI / I²C to the RP2040 nodes instead of SPI",
             risk="linearity / noise before calibration differ; node firmware changes (it does not exist yet, Fable M3); the 0.9 mm hand accuracy claim must be re-shown",
             checks="rnd.accuracy with the MT6701 figures after a calibration table; encoder bracket fit (rnd/parts/encoder.py)", own="sensors / sw track",
             gates="none (accuracy is not a gate; R2 / R17 / A10 / thermal untouched)"),
        dict(t="Bond the inline roll's 61806 outer race with the structural epoxy already bought (drop the Loctite 660 tube)", s=28.99,
             change="the 61806 floats ± 0.3 mm and is bonded in situ: use the EA 9466 / DP420 of the elbow-core line (gap-filling) instead of Loctite 660; 638 stays for the dowels",
             risk="service removal (heat) and creep under the ≤ 800 N misalignment preload; anaerobic 660 is the datasheet choice for bearing seats",
             checks="rnd.joints roll-support row (bond shear at 800 N), a bonded-race coupon (COUPONS.md)", own="roll track (rnd/parts/roll.py notes) + joints.py",
             gates="R2 (roll support row must re-pass)"),
        dict(t="One caster-release solenoid pulling 4 Bowden cables (D18b)", s=3 * K.SOLENOID[0] - 12.0,
             change="4 × 24 V solenoids → 1 solenoid + 4 Ø1 steel cables in PTFE liners (≈ $12 ESTIMATE); the locks stay spring-applied, so any failure leaves all four locked",
             risk="force budget untested (friction of four cables); one failure releases or locks all four at once (locked is the safe side)",
             checks="bench pull-force test of the cable release; A10 rnd.tip unchanged (the lock geometry does not move); sw SW-T2 caster-lock output (one output instead of four)",
             own="base track (rnd/parts/base.py) + sw", gates="A10 (re-run, expected unchanged)"),
        dict(t="Generic 304 3\" quick-release tri-clamps for the arm detach (2)", s=2 * (25.0 - 12.0),
             change="Dixon 13MHHM-Q300 ($25) → a generic 304 SS 3\" quick-release squeeze clamp (≈ $12 ESTIMATE) on the same 90.9 mm flange",
             risk="clamp force and fit vary between makers; a loose clamp lets the arm rock at the detach",
             checks="KX-V flange OD 90.9 mm fit, clamp preload on the ferrule (bump / stall wrench rows in joints.md), R3 split/join ≤ 30 s",
             own="arm track (shoulder.py KX-V ferrule)", gates="R2 (detach wrench rows)"),
    ]


# --- D29 (owner 2026-11-28: no belt roll; the lead's route for the rest): what each row became -----------------------------------
D29_BEFORE, D29_BEFORE_D20 = 6713.0, 6888.0     # bom.json before D29 (W14c)
W15_BEFORE, W15_BEFORE_D20 = 6366.16, 6632.96    # bom.json after D29 (W15a), before the W15b base-channel and W14b pinch lines


def landed(I):
    """Landed total of a line list without the contingency row: lines + the rounded 15 % (as report() adds it)."""
    sub = sum(q * u for _, _, q, u, _ in I)
    return sub + round(sub * CONTINGENCY, 0)


def timer_totals(d=BASELINE):
    """{PIP_BASE_TIMER option: dict(total, with_d20, unit)}: the robot total with each U20 OFF-delay timer (owner decision pending)."""
    if not HOLDS:
        HOLDS.update(holds())
    if id(d) not in _TT:                                    # items() takes ≈ 3 s (the accuracy score): once per design
        _TT[id(d)] = {k: landed(items(d, timer=k)) for k in K.BASE_TIMERS}
    return {k: dict(total=_TT[id(d)][k], with_d20=_TT[id(d)][k] + HOLDS["d20"], unit=v[0]) for k, v in K.BASE_TIMERS.items()}


_TT = {}


def w15_block(I, tot, d=BASELINE):
    """Report lines for the W15b base channel + W14b pinch parts and the pending base-timer decision."""
    c = 1 + CONTINGENCY
    TT = timer_totals(d)
    new = [(n, q, u, src) for _, n, q, u, src in I if n.startswith(("Base channel (W15b)", "W14b "))]
    w15 = sum(q * u for n, q, u, _ in new if n.startswith("Base channel"))
    w14 = sum(q * u for n, q, u, _ in new if n.startswith("W14b"))
    L = [f"**W15b base channel + W14b pinch parts (2026-11-28): ${W15_BEFORE:,.0f} → ${tot:,.0f} landed ({_sd(tot - W15_BEFORE)}); with D20 "
         f"${W15_BEFORE_D20:,.0f} → ${tot + HOLDS['d20']:,.0f}.** The base moved off K1 for the category-1 base stop (electrical.md *Base channel*, "
         f"docs/sw/base_stop.md): {_sd(w15)} of lines ({_sd(w15 * c)} landed) with the `{K.BASE_TIMER}` timer; the W14b roll front collars' 8 × M4 × 25 "
         f"{_sd(w14)} of lines, their PETG (net ≈ +{W14B['kg'] * 1000:.0f} g) inside the PETG bundles already bought (no extra bundle). Robot only: the M1 kit "
         "changes only in its Step-1 test kit (M1_BENCH_ARM.md).", "",
         f"**Owner decision pending: the U20 2.0 s OFF-delay timer** (`PIP_BASE_TIMER`; the BOM uses `{K.BASE_TIMER}`, provisionally):", "",
         "| PIP_BASE_TIMER | U20 | Unit $ | Robot (landed) | With D20 | |", "|---|---|---|---|---|---|"]
    for k, v in TT.items():
        L.append(f"| `{k}`{' (provisional default)' if k == 'board' else ''} | {K.BASE_TIMERS[k][1]} | {v['unit']:,.2f} | **${v['total']:,.0f}** | "
                 f"${v['with_d20']:,.0f} | {'in the total' if k == K.BASE_TIMER else _sd(v['total'] - TT[K.BASE_TIMER]['total']) + ' landed'} |")
    L += ["", "| New line | Qty | Unit $ | Line $ | Source |", "|---|---|---|---|---|"]
    L += [f"| {n} | {q} | {u:,.2f} | {q * u:,.2f} | {src} |" for n, q, u, src in new]
    L += [""]
    return L
HAND_TORQUE = (0.3, 1.0)        # N·m: a hand-tight wing nut, a light to a firm hand (ESTIMATE)
WING_BOLT_D, NUT_K = 0.0079, 0.20   # 5/16" T-bolt of a 3" single-hinge clamp; nut factor (ESTIMATE)
MU_CLAMP = (0.15, 0.30)         # clamp ↔ ferrule flank and band friction (ESTIMATE)
# Arm split / join step times (s per arm, ESTIMATE; R3: split/join ≤ 30 s without tools). The blind-mate XT60 + pogo plug
# floats beside the ferrule, so it parts and mates with the flange (electrical.md); the arm is lifted off / seated on the dowels.
SPLIT_STEPS = {"dixon": dict(split=[("open the squeeze lever, swing the clamp off", 2.0), ("lift the arm off the dowels", 3.0)],
                             join=[("seat the arm on the 3 dowels", 4.0), ("clamp on, squeeze the lever shut", 3.0)]),
               "wingnut": dict(split=[("loosen the wing nut ≈ 2 turns", 3.0), ("swing the T-bolt out, open the clamp", 2.0), ("lift the arm off", 3.0)],
                               join=[("seat the arm on the 3 dowels", 4.0), ("close the clamp, swing the T-bolt in", 2.0),
                                     ("run the wing nut ≈ 2 turns, hand-tight + a firm quarter turn", 6.0)])}


def detach_recheck():
    """D29 #10: KX-V clamp preload needed by the detach rows (rnd/kx_check.py: lift-off ≥ 1.5 × the dynamic moment) against each
    clamp's preload, and the two-arm split / join time. Wing nut: band tension = the T-bolt load T = τ / (K d), averaged over
    the hinged band by the capstan factor (1 − e^(−μπ)) / (μπ), axial = 2π T_avg (cos α − μ sin α) / (sin α + μ cos α) on the
    20° sanitary flank. The squeeze clamp's preload is kx_check's 1,000 N ESTIMATE (set by its geometry)."""
    from .kx_check import VBAND, detach_check
    r = detach_check()
    p_need = 1.5 * r["m_need"] / (VBAND["contact_r"] / 1000)
    a = math.radians(VBAND["groove_half_angle"])

    def axial(tq, mu):
        T = tq / (NUT_K * WING_BOLT_D)
        return 2 * math.pi * T * (1 - math.exp(-mu * math.pi)) / (mu * math.pi) * (math.cos(a) - mu * math.sin(a)) / (math.sin(a) + mu * math.cos(a))
    wn = [axial(tq, mu) for tq in HAND_TORQUE for mu in MU_CLAMP]
    pre = {"dixon": (VBAND["preload_n"], VBAND["preload_n"]), "wingnut": (min(wn), max(wn))}
    t_min_wing = HAND_TORQUE[0] * p_need / min(axial(HAND_TORQUE[0], mu) for mu in MU_CLAMP)
    out = {}
    for k, (lo, hi) in pre.items():
        ts, tj = (2 * sum(t for _, t in SPLIT_STEPS[k][w]) for w in ("split", "join"))
        bear = hi / (2 * math.pi * VBAND["flange_od"] / 2 * 4.0)
        out[k] = dict(preload_lo=lo, preload_hi=hi, margin_lo=lo / p_need, bearing_hi=bear, t_split=ts, t_join=tj,
                      ok=lo >= p_need and ts <= 30.0 and tj <= 30.0 and bear <= 12.0)
    return dict(p_need=p_need, m_need=r["m_need"], t_min_wing=t_min_wing, clamps=out)


def d29_rows(I):
    """[(#, item, status, check result, landed saving counted)] per cost_down row, from the checks' own outputs."""
    c = 1 + CONTINGENCY
    th = json.loads((DOCS / "thermal.json").read_text()).get("d29_stock_cap", {}) if (DOCS / "thermal.json").exists() else {}
    tp = json.loads((DOCS / "tip.json").read_text()).get("d29_caster_release", {}) if (DOCS / "tip.json").exists() else {}
    from . import accuracy as A
    ec = A.encoder_compare()
    dr = detach_recheck()
    n_gim = sum(q for _, n, q, _, _ in I if n == "GIM6010-8")
    on = lambda flag: "**adopted**" if flag else "not adopted"
    s_cap = 6 * (K.AL_CAP_MACHINED[0] - K.AL_CAP_STOCK[0])
    cap_ok = bool(th.get("ok")) and s_cap > 0
    enc_ok = ec["mt6701"][0] <= ec["as5048a"][0] + 0.05
    cab = tp.get("variants", {})
    cas_ok = bool(tp.get("a10_ok")) and bool(tp.get("force_timing_ok"))
    s_cas = 4 * K.SOLENOID[0] + 6.0 - (K.SOLENOID_55N[0] + K.CABLE_KIT[0] + 4.0)
    w = dr["clamps"]["wingnut"]
    s_cl = 2 * (K.DETACH_CLAMPS["dixon"][0] - K.DETACH_CLAMPS["wingnut"][0])
    rows = [
        (1, "Belt shoulder roll", "skipped (owner, D29: \"I don't love more belts\")", "—", 0.0),
        (2, f"GIM6010-8 at the $70 listing ({n_gim} units)", on(K.GIM6010[0] == 70.0) + " (sourcing only)",
         f"Robotics Center $70, SDC104 24 V, backordered (2026-11-28); fallback aifitlab $90 = +${n_gim * 20:.0f} lines (+${n_gim * 20 * c:,.0f} landed) "
         "if the backorder holds the buy; Step 1's one unit is priced at the fallback", n_gim * 20.0 if K.GIM6010[0] == 70.0 else 0.0),
        (3, "4130 pitch sleeves", "skipped (D29: stall margin 1.18× → 1.08× too thin)", "—", 0.0),
        (4, "Batch metal quote", "note only (not in totals until quoted)", f"≈ ${next(r['s'] for r in savings(I) if r['t'].startswith('One batch quote')) * c:,.0f} landed at −15 % of the custom metal (ESTIMATE, "
         "no quote yet)", 0.0),
        (5, "Stock round heatsinks for the 6 rear caps", on(cap_ok and K.REAR_CAP == "stock") + " (PIP_REAR_CAP=stock)",
         f"thermal re-check {'PASS' if th.get('ok') else 'FAIL'} (thermal.md §4: {th.get('new_hi', '?')} rows newly over 100 °C in the band, "
         f"band worst up to {th.get('d_hi_max', 0):+.0f} K; task day {th.get('task_day_machined', 0):.0f} → {th.get('task_day_stock', 0):.0f} °C nominal); and the sourced "
         f"price is ${K.AL_CAP_STOCK[0]:.2f} each vs the CNC cap's ${K.AL_CAP_MACHINED[0]:.0f} ESTIMATE ({_sd(-s_cap * c)} landed, the $6 was a wholesale figure)", 0.0),
        (6, "MT6701 encoders (7 shaft-end joints)", on(enc_ok and K.JOINT_ENCODER == "mt6701") + " (PIP_JOINT_ENCODER=mt6701)",
         f"hand error with joint encoders {ec['as5048a'][0]:.1f} → **{ec['mt6701'][0]:.1f} mm** (band {ec['mt6701'][1]:.1f}–{ec['mt6701'][2]:.1f}; datasheet INL "
         "±1.0° typ / ±1.5° max vs the AS5048A's ±0.8°, no MT6701 noise figure found); the D9 0.9 mm claim is not re-shown; would save "
         f"${7 * (12.0 - 4.0) * c:,.0f} landed (MT6701 boards ≈ $3–7)", 7 * (12.0 - K.ENCODER_P[0]) if K.JOINT_ENCODER == "mt6701" else 0.0),
        (7, "NH-50 clamp resistors at a distributor price", "**adopted** (ESTIMATE kept)",
         f"NH050R5600FE02; no 0.56 Ω listing found, NH-50 single-unit listings $12–20 → ${K.NH50[0]:.0f} ESTIMATE (was $30): 3 × ${30 - K.NH50[0]:.0f}", 3 * (30.0 - K.NH50[0])),
        (8, "Roll 61806 bonded with the epoxy", "skipped (D29: rnd/parts/roll.py is another engineer's)", "—", 0.0),
        (9, "One caster solenoid pulling 4 cables", on(cas_ok and K.CASTER_RELEASE == "cable") + " (PIP_CASTER_RELEASE=cable)",
         f"A10 {cab.get('four', {}).get('worst', 0) * 1000:.0f} → {cab.get('cable', {}).get('worst', 0) * 1000:.0f} mm, {cab.get('cable', {}).get('n_ok', '?')}/16; "
         f"force + timing {'PASS' if tp.get('force_timing_ok') else 'FAIL'} only with a 55 N JF-1264B (the 25 N JF-1039 is 0.84× on 4 cables), release ≤ 31 ms / re-lock "
         f"≤ 22 ms vs the 100 ms re-seat pulse (tip.md D29 #9); saving shrinks to ${s_cas:.0f} lines; a stuck plunger now leaves all four released",
         s_cas if (cas_ok and K.CASTER_RELEASE == "cable") else 0.0),
        (10, "Generic 3\" clamps for the arm detach", on(w["ok"] and K.DETACH_CLAMP == "wingnut") + " (PIP_DETACH_CLAMP=wingnut)",
         f"no generic quick-release squeeze clamp sourced below the Dixon's $25 (Steel & O'Brien squeeze $46.39); the generic wing-nut 3\" clamp ($10.40): "
         f"preload {w['preload_lo'] / 1000:.1f}–{w['preload_hi'] / 1000:.1f} kN vs {dr['p_need']:.0f} N needed ({w['margin_lo']:.1f}× at a light hand, "
         f"fails below ≈ {dr['t_min_wing']:.2f} N·m), flank {w['bearing_hi']:.1f} MPa on PETG; two-arm split {w['t_split']:.0f} s / join {w['t_join']:.0f} s ≤ 30 s (ESTIMATE)",
         s_cl if (w["ok"] and K.DETACH_CLAMP == "wingnut") else 0.0),
    ]
    return rows, dr


def cost_down_report(I, tot):
    """docs/rnd/cost_down.md: where the money goes, the top savings, what was rejected."""
    c = 1 + CONTINGENCY
    sub = subsystems(I)
    lines = sum(sub.values())
    S = sorted(savings(I), key=lambda r: -r["s"])
    safe = [r for r in S if not r.get("flag")]
    s_safe = sum(r["s"] for r in safe) * c
    s_all = sum(r["s"] for r in S) * c
    d20 = HOLDS["d20"]
    top = lambda subname: ", ".join(f"{n[:48]}{'…' if len(n) > 48 else ''} ${v:,.0f}" for n, v in sorted(
        ((n, sum(x for k, x in _split(n, q, u) if k == subname)) for c_, n, q, u, _ in I if not c_.startswith(("Contingency", "Docking"))),
        key=lambda kv: -kv[1])[:3] if v > 0)
    kt = K.totals()
    bench = ("Mean Well", "bench supply", "Bench stand", "Test lever", "Category-0 catch", "sw/bringup.py", "CAN FD HAT", "PETG 3 kg",
             "Epoxy (CF bonds)", "Stainless screw kit")
    m1 = K.step1() + K.step2_common() + K.HOLD_A["no"] + K.HOLD_B["yes"]
    reuse = sum(q * u for _, n, q, u, _, _ in m1 if not any(b in n for b in bench)) * c
    L = ["# Cost-down list (generated by `python -m rnd.bom`; prices are list / ESTIMATE, nothing bought)", "",
         f"**Robot BOM ${tot:,.0f} landed** (${lines:,.0f} of lines + {CONTINGENCY:.0%} contingency), **${tot + d20:,.0f} with D20** (GIM8108-8 at pitch + elbow, "
         f"{_sd(d20)}), with the W15b base timer U20 = `{K.BASE_TIMER}` (owner decision pending; "
         + "; ".join(f"`{k}`: ${v['total']:,.0f} / ${v['with_d20']:,.0f} with D20" for k, v in timer_totals().items()) + f"). Limit $5,500, aim $4,000 (R8); owner decision D26: cost-down redesign first. Gap: {_sd(tot - 5500)} ({_sd(tot + d20 - 5500)} with D20) to the "
         f"limit, {_sd(tot - 4000)} to the aim. Line detail: [BOM_v4.md](BOM_v4.md).", ""]
    D, dr = d29_rows(I)
    s29 = sum(r[4] for r in D) * c
    TT = timer_totals()
    tt_alt = "; ".join(f"`{k}` ${v['total']:,.0f} / ${v['with_d20']:,.0f} with D20" for k, v in TT.items() if k != K.BASE_TIMER)
    kt0 = K.totals()
    L += ["## D29 outcome (2026-11-28)", "",
          f"Owner: \"I don't love more belts. You can decide the best route for the other stuff\"; the lead's route: sourcing-only rows now, the rest only where "
          f"its re-check passes (each behind a params switch). **${D29_BEFORE:,.0f} → ${W15_BEFORE:,.0f} landed ({_sd(W15_BEFORE - D29_BEFORE)}); with D20 ${D29_BEFORE_D20:,.0f} → "
          f"${W15_BEFORE_D20:,.0f}** (D20's delta grows {_sd(d20 - (D29_BEFORE_D20 - D29_BEFORE))} because the GIM6010s it replaces got cheaper). "
          f"Rows counted: −${s29:,.0f} landed. Since then the W15b base channel and the W14b pinch parts: **${tot:,.0f}** ({_sd(tot - W15_BEFORE)}, "
          f"U20 `{K.BASE_TIMER}` timer, owner decision pending; {tt_alt}), {_sd(tot - 5500)} against the $5,500 limit ({_sd(tot + d20 - 5500)} with D20). "
          f"M1: Step 1 ${kt0['s1_c']:,.0f}, Step 2 ${kt0['s2_plan_c']:,.0f} (kit ${kt0['plan_c']:,.0f}).", "",
          "| # | Row | D29 | Re-check / sourcing result | Counted (landed) |", "|---|---|---|---|---|"]
    L += [f"| {i} | {t} | {st} | {ck} | {f'−${s * c:,.0f}' if s else '—'} |" for i, t, st, ck, s in D]
    L += ["", "Switches (rnd/params.py; a default changes only when its row passes): `PIP_REAR_CAP` machined | stock, `PIP_JOINT_ENCODER` as5048a | mt6701, "
          "`PIP_CASTER_RELEASE` four | cable, `PIP_DETACH_CLAMP` dixon | wingnut; W15b (read in rnd/m1_kit.py, BOM only, owner decision pending): `PIP_BASE_TIMER` board | g9sa. Arm split / join steps (ESTIMATE, s per arm): "
          + "; ".join(f"{k}: split " + " + ".join(f"{n} {t:g}" for n, t in v["split"]) + ", join " + " + ".join(f"{n} {t:g}" for n, t in v["join"])
                      for k, v in SPLIT_STEPS.items()) + ".", ""]
    L += ["## Where the money goes", "",
         "| Subsystem | Lines $ | Landed $ | Share | Largest lines |", "|---|---|---|---|---|"]
    for k in SUBSYS:
        v = sub[k]
        L.append(f"| {k} | {v:,.0f} | {v * c:,.0f} | {v / lines:.0%} | {top(k) if k != SUBSYS[8] else 'charger, contacts, electronics, baseplate'} |")
    L += [f"| **Total** | **{lines:,.0f}** | **{lines * c:,.0f}** | 100 % | (contingency {CONTINGENCY:.0%} spread over every subsystem) |", "",
          f"- The two arms are {(sub[SUBSYS[0]] + sub[SUBSYS[1]]) / lines:.0%} of the robot; actuators alone (arms + base + lift + head) are "
          f"{sum(q * u for c_, n, q, u, _ in I if c_ == 'Actuators') / lines:.0%}. D20 adds {_sd(d20)} to the arm actuators.",
          "- The inline-arm study ([inline_study.md](inline_study.md)) settles the direction: a D26 belt stage is ≈ $60–70 of parts, and every inline swap "
          "costs $45–380 more per joint (all-inline +$585–1,068 landed over B1; B1y's inline yaw +$100). There is no cost-down in more inline motors; the "
          "only inline joint, the roll, is the largest single owner trade below.",
          "- Competitors ([competitors.md](competitors.md)): XLeRobot reaches ≈ $660 with STS3215 servos and one-handed payloads; Pip's money is in the 20 lb "
          "two-handed carry (R2c / R2d) and the safety chain, which the gates keep.", "",
          "## Top 10 savings (the W14c list as written, its estimates; D29's outcome per row is the table at the top)", "",
          "Ranked by landed saving. Row 1 is the queued belt-roll idea: it is the only large one and it is **not** gate-neutral (flagged); rows 2–10 keep "
          "R2 strength, R17 pinch, A10 tip and the thermal rows passing (each with the checks it needs before it lands).", "",
          "| # | Saving (landed) | What changes | Risk | Checks it needs | Owner | Gates touched |", "|---|---|---|---|---|---|---|"]
    for i, r in enumerate(S, 1):
        L.append(f"| {i} | **{r['s'] * c:,.0f}** ({r['s']:,.0f} lines) | **{r['t']}:** {r['change']} | {r['risk']} | {r['checks']} | {r['own']} | {r['gates']} |")
    L += ["", f"**As written (W14c, on ${D29_BEFORE:,.0f}): rows 2–10 together −${s_safe:,.0f} landed → ${D29_BEFORE - s_safe:,.0f}; with row 1 "
          f"${D29_BEFORE - s_all:,.0f}.** D29 counted −${s29:,.0f} of it (top table): the row-1 belt roll is off (owner), rows 3 / 8 skipped, row 4 is a "
          "note, and rows 5 / 6 did not pass; rows 9 and 10 passed at a smaller or different part than the list assumed. "
          "The gap to $5,500 is carried until the next cost-down idea (an owner trade: D20's scope or the D9 sensor pack).", "",
          "## Considered and not recommended", "",
          "| Idea | Saving | Why not |", "|---|---|---|",
          "| All-inline arm (I1 / I2a / I2b) | −(+$585…1,068) | costs more, misses A10 or the hug at 0.25 m (inline_study.md, Astra #3 Part A) |",
          "| B1y inline DM-J4340 yaw | −(+$100) | a cost increase; adopt only after a returnable sample passes (inline_study.md) |",
          "| Pinion v5 → v4 stub + stock keyed pulley | ≈ $15 | the keyed Ø12 is 0.68× (D20) / 1.19× (GIM6010) in rotating-bending fatigue (belts.md, Astra #3 m7) |",
          "| One arm | ≈ $1.3k | fails R2c / R2d (two-handed 20 lb) |",
          "| Base redesign (3-wheel kiwi / hub motors) | ≈ $90–300 | D2: kiwi tip margin 35–46 vs 66–82 mm (A10); no omni-compatible hub motor sourced |",
          "| Two clamp resistors instead of three (robot regen limit 1,663 W) | ≈ $40 | the clamp is the bus's only over-voltage protection; its margin is a safety function |",
          "| JK-B1A8S10P 100 A BMS | ≤ $19 | $76–117 listings vs $95; halves the margin over the 74 A peak |",
          "| DIY 6S2P pack, cheaper DC contactor, a lower contingency | $50–80 each | home-robot safety (unrated welds / contactor), and this pass found +40 % on single lines |",
          f"| Sensor pack v2 (encoders, nodes, tactile, ToF) | ≈ ${sum(q * u for c_, _, q, u, _ in I if c_ == 'Sensor pack v2') * c:,.0f} | owner D9: sensors over savings (the encoders carry the 0.9 mm accuracy) |",
          "| One PETG bundle | $110 | the print plan needs 17.8 kg with 20 % spare; one bundle needs ≤ 8.3 kg printed |", "",
          "## Cash view: the M1 kit is not extra", "",
          "The M1 bench kit's arm and safety-board parts are the robot's (the bench board moves to the robot; the motors, bearings, pinions and hardware "
          "of one arm are one of the robot's two). The program's spend is the M1 kit plus the robot BOM less what M1 already bought; only the bench "
          "supplies, the stand, the lever, the catch, the coupons and the Step-1 test kit stay on the bench ([M1_BENCH_ARM.md](../M1_BENCH_ARM.md)).", "",
          f"- M1 kit, today's plan, with contingency: ${kt['plan_c']:,.0f} (Step 1 ${kt['s1_c']:,.0f} incl. the test kit ${kt['s1_test_c']:,.0f}, Step 2 ${kt['s2_plan_c']:,.0f}).",
          f"- Of it, robot parts (ESTIMATE: every line except the bench-only ones and the test kit): ≈ ${reuse:,.0f} → the robot after M1 needs ≈ ${tot - reuse:,.0f} more, "
          f"and the program total is ≈ ${kt['plan_c'] + tot - reuse:,.0f} (≈ ${kt['plan_c'] - kt['s1_instr_c'] + tot - reuse:,.0f} if the owner already has the test instruments).", ""]
    return "\n".join(L) + "\n"


def report(d=BASELINE):
    HOLDS.update(holds())
    pp = json.loads((DOCS / "print_plan.json").read_text()) if (DOCS / "print_plan.json").exists() else {"kg": 10.4}
    I = items(d)
    sub = sum(q * u for _, _, q, u, _ in I)
    I.append(("Contingency (A16)", f"{CONTINGENCY:.0%} for price drift, shipping and tax", 1, round(sub * CONTINGENCY, 0), "review finding 18"))
    tot = sum(q * u for _, _, q, u, _ in I)
    cats = {}
    for c, _, q, u, _ in I:
        cats[c] = cats.get(c, 0) + q * u
    L = ["# Bill of materials v4 (auto-generated by `python -m rnd.bom`)", "",
         f"**Total ≈ ${tot:,.0f} landed** (${sub:,.0f} + {CONTINGENCY:.0%} contingency; target ≤ $4,000 since D9 (owner: sensors over savings); limit $5,500 (owner, 2026-11-27; was $5,000); excludes the owner's Pi 5 and tools). Single-unit online prices; hardware lines are ESTIMATEs.", "",
         ] + w15_block(I, tot, d) + [
         f"**Waves 12–13 refresh (2026-11-28, the merged design after Astra review #3): ${W12_TOTAL:,.0f} → ${W15_BEFORE:,.0f} landed ({_sd(W15_BEFORE - W12_TOTAL)}, "
         f"D29 included); with D20 ${W15_BEFORE_D20:,.0f}.** Where the money goes, the top-10 savings and the **D29 outcome** (${D29_BEFORE:,.0f} → ${W15_BEFORE:,.0f}): "
         "[cost_down.md](cost_down.md). Line changes (before → after, $ of lines, no contingency):", "",
         "| Line | Before $ | After $ | Source / check |", "|---|---|---|---|"] + [
         f"| {w} | {b:,.0f} | {a:,.0f} | {src} |" for w, b, a, src in REFRESH_ROWS] + [
         "", f"Refresh lines {_sd(sum(a - b for _, b, a, _ in REFRESH_ROWS))} before contingency. Step-1 test equipment (scope, probes, the 20 A CC supply, dummy banks) "
         "is tooling: it is in the M1 kit, not here.", "",
         "**Options (priced, not in the total; owner decision pending, CAD behind a params switch):**", "",
         "| Option | Qty | Unit $ | Line $ | Landed $ | Source | Note |", "|---|---|---|---|---|---|---|"] + [
         f"| {n} | {q} | {u:,.2f} | {q * u:,.0f} | {q * u * (1 + CONTINGENCY):,.0f} | {src} | {why} |" for n, q, u, src, why in options(d)] + ["",
         "**Loop 47 cost pass** (motor choices unchanged; before → after, each with the 15 % contingency):",
         "- A5 v2 safety electrics added first: DC contactor with aux contact, switched pre-charge, energy-rated regen clamp: $71 → $208 of lines (+$158 landed): $5,030 → $5,188.",
         "- Savings (−$290 landed): PETG in 10 kg bundles ($333 → $220); five $19 candleLight USB-CAN adapters in place of the CAN HAT + three $35 adapters "
         "($152 → $95); one USB-TTL adapter instead of three (the D14 nodes drive the buses); no separate balance charger (the dock's CC/CV charger + the BMS's active "
         "balancing do the weekly 4.20 V charge, A13); GIM6010-8 at the verified $90, DM-J4310 at $150 in stock, RPLIDAR C1 at $69.",
         "- Result: **$4,898**, under the $5,000 limit; the $4,000 target would need the motor or sensor choices reopened.", "",
         f"**Wave 1 (D24 bolted cartridges, D19 one-piece shafts; 2026-11-27): ${WAVE1_BEFORE:,.0f} → ${WAVE1_AFTER:,.0f} landed.** The arm's 10 KX sockets go "
         "(balls, dowel packs, springs, torque pins: −$35 of lines); in: 8 Oldham couplings ($96; the spare acetal rod is an M1 bench item), heat-set inserts ($44), flange "
         "screws ($10), Ø6 dowels for the yaw stops and ferrules ($6); the D13 flange couplings go (−$24); D14 buck 3 → 6 A (+$4); PETG from the "
         "re-sliced print plan. +$13 against the $5,000 limit at the time. The owner's OK for the inline GIM8108-36 roll then put two GIM8108-36 "
         f"in place of two GIM6010-8 (at the yaml's $171): ${WAVE2_BEFORE:,.0f}.", "",
         f"**Fable review fixes (2026-11-27, M2 + M7): ${WAVE2_AFTER:,.0f} → ${NEON2_BEFORE:,.0f} landed ({_sd(NEON2_BEFORE - WAVE2_AFTER)}).** A third clamp resistor + MOSFET "
         "(the inline roll raised the robot's peak regen to 2.8 kW), the safety-board logic the netlist needs (one-shot discharge, latch, comparators), and "
         "aluminium finned rear caps + thermistors on the six arm GIM6010 cartridges (a closed ASA sleeve allows only 0.65 × rated continuous). The C1/M1 "
         "joint hardware (tie-rods, 6061 yaw flange, 7075 crank) was in the M1 kit only until W10 (below).", "",
         f"**Neon review #2 fixes (2026-11-27; findings 1, 10, 11, 12 and the keyed D50 drive of 2/15): ${NEON2_BEFORE:,.0f} → ${W8_BEFORE:,.0f} landed ({_sd(W8_BEFORE - NEON2_BEFORE)}).** "
         "NH-50 clamp resistors + freewheel Schottkys (+$51), a 200 A BMS for the 40 A class (+$50), a steel battery box (+$20), 60 A main fuse (+$10), XT90 / XT60 / USB pogo "
         "connectors (+$12), the lift umbilical (+$30), 16 AWG fluoropolymer ribbon wire (+$10), the dock baseplate and taller pads instead of floor screws (+$14), "
         "keyed stages and D50 Oldhams instead of plain shafts and D40s (+$111), the netlist-v3 logic (+$5); plus contingency.", "",
         f"**W10 consolidation of the wave-8 hardware (2026-11-27; W8b cartridge, W8c load path, W8d tip): ${W8_BEFORE:,.0f} → ${D26_BEFORE:,.0f} landed "
         f"({_sd(D26_BEFORE - W8_BEFORE)}), {_sd(D26_BEFORE - 5500)} against the $5,500 limit.** The D50 Oldham was a $20 generic ESTIMATE with no source, keyway or torque "
         "rating; it is now priced as the parts the cartridge calls for (Ruland MOCC51-12-A keyed clamp hub $67.19 + OD32/51-AT acetal disc $36.45 at Zoro) with a "
         "6061 motor-side hub (the printed ASA tongue was rejected) on all 7 GIM6010 stages (the lift's was missing), 2 spare discs; 4-ear flange screws with nuts "
         "replace the countersunk M4s; turned 42CrMo4 crush sleeves at every C1 tie / clevis / rear-mount bolt + 2 stop bushes, the 7075 rear plate, M4 × 25 rear "
         "screws and Loctite 660 / 638 (no Belleville springs; the M4 × 18 + shims and the Ø10 stop dowels go); the elbow's 6061 core tube; D18b lockable 2\" casters "
         "with solenoid release and the 2 × 2.44 kg ballast (W8d's ≈ $80 ESTIMATE was ≈ $410 of lines priced); and the C1 joint hardware (tie-rods, 6061 yaw "
         "flange, 7075 crank) that was in the M1 kit only. Largest drivers and the cheapest credible alternative for each:", "",
         "| Added today (landed, with contingency) | $ | Cheapest credible alternative | Saves |", "|---|---|---|---|"]
    DR = drivers(d)
    L += [f"| {w} | {a:,.0f} | {alt} | {s:,.0f} |" if s else f"| {w} | {a:,.0f} | {alt} | — |" for w, a, alt, s in DR]
    sv = sum(r[3] for r in DR)
    c = 1 + CONTINGENCY
    d26 = sum(b - a for _, b, a, _ in D26_ROWS) * c
    L += ["", f"The alternatives still open together: −${sv:,.0f} → ${tot - sv:,.0f} ({_sd(tot - sv - 5500)} against the $5,500 limit).", "",
          f"**D26 cost-down, wave 11 (owner: redesign first, $5,500 target): sleeves, casters, ballast −${d26:,.0f} landed** (lines before → after, "
          "the checks they pass before → after):", "",
          "| Item | Before $ | After $ | Checks |", "|---|---|---|---|"]
    L += [f"| {w} | {b:,.0f} | {a:,.0f} | {ck} |" for w, b, a, ck in D26_ROWS]
    L += ["", f"Total ${D26_BEFORE:,.0f} → **${tot:,.0f}** landed (this table and any other wave-11 change merged in the tree), {_sd(tot - 5500)} against the $5,500 limit. "
          "Stop bushes and the pitch / yaw sleeves stay turned (no stock part fits the CAD holes at their stall stress).", ""]
    GT = gap_table(I[:-1])
    gs = sum(r[4] for r in GT)
    L += [f"**What is left of the gap ({_sd(tot - 5500)}): the largest lines by landed cost** (with contingency) and the cheapest credible alternative "
          "for each; the last row is the next line that has one:", "",
          "| Line | Qty | Landed $ | Cheapest credible alternative | Saves |", "|---|---|---|---|---|"]
    L += [f"| {nm} | {q} | {v:,.0f} | {a} | {f'{sv:,.0f}' if sv else '—'} |" for nm, q, v, a, sv in GT]
    L += ["", f"Together −${gs:,.0f} → ${tot - gs:,.0f} ({_sd(tot - gs - 5500)} against the $5,500 limit); the belt-roll row is an owner decision "
          "(it reverses the inline-roll choice).", "",
         f"**Wave-2 cost pass (2026-11-27; motor choices and safety items unchanged): ${WAVE2_BEFORE:,.0f} → ${WAVE2_AFTER:,.0f} landed ({_sd(WAVE2_AFTER - WAVE2_BEFORE)}).** "
         "The inline roll was carried as a bare motor with the belt-roll parts still in the list, so corrections come first (lines, before contingency):",
         "- GIM8108-36 $171 → **$240** with its 24 V driver (SteadyWin store $239.80 GDM810 = SDC303; aifitlab $240 GDK810; $190 is the bare motor): +$138.",
         "- Belt-roll parts out (2 Oldham, 2 D13 stages, 2 belts, 4 × 61808, 4 × 6801: −$72); inline parts in (2 × 7075 output adapter at a $30 CNC "
         "ESTIMATE, 2 × 61806, steel clamp ring + screws + dowels: +$88). A printed adapter is not an option (3 × M4 into PETG ≈ 85 N·m < 103 N·m stall).",
         "- Sensor nodes 3 → 5 (sw/config/can_map.yaml has a shoulder and an elbow node per arm): +$20.",
         "- Savings: DM-J4310 $150 → **$125** (AIFit Lab, 24 V, US seller, backordered: the full-robot buy comes after M1): −$50; D23 covers $60 → $30 "
         f"(the rigid covers, 1.4 kg of PETG, fit in the 10 kg bundles: 18.2 kg needed with spare of 20 kg bought; TPU 95A + fasteners stay): −$30.",
         "- Not taken: Foxtech DM-J4310 $116–120 (ships from China: tariff/duty exposure eats the gap); pultruded CF (higher axial E, but the forearm's "
         "double-D pinch clamp and 1 mm wall split a unidirectional tube; roll-wrapped listings are already $63–70); DIY 6S2P from cells (≈ −$50, but a "
         "home-welded pack without a spot welder or cell matching is a safety risk in a home robot); generic D40 Oldham (no sourced price under $12; Ruland/GMT $36–67); cheaper "
         "heat-set inserts (the short M4 is $11–21 / 50 elsewhere); a lower contingency (this pass found a +40 % price on one line, and China-direct "
         "parts now pay duty: 15 % stays); Gemini 2 / RPLIDAR C1 / battery / GIM6010-8 are already at the best sourced price.",
         f"- **{_sd(tot - 5500)} against the $5,500 limit** (owner raised it from $5,000 on 2026-11-27 to keep the inline roll and the review fixes). The belt roll (fallback `PIP_ROLL_DRIVE=belt`) is "
         f"{_sd(-HOLDS['inline'])} landed; the owner chose the inline roll for its hand-feel and slimmer shoulder.", "",
         "**Pending decisions (not in the total; the M1 kit's holds, robot-wide, with contingency):**",
         f"- **U20 base OFF-delay timer (W15b, owner):** `PIP_BASE_TIMER={K.BASE_TIMER}` is in the total provisionally; "
         + "; ".join(f"`{k}` ${v['total']:,.0f} (${v['with_d20']:,.0f} with D20)" for k, v in timer_totals(d).items())
         + " (the G9SA is a certified relay with a 13-week lead; the board pair is ± 20 % and not rated).",
         f"- **D20 GIM8108-8 at shoulder pitch + elbow** (waits on the seller's 24 V answer): 4 × GIM6010-8 → 4 × GIM8108-8: "
         f"**{_sd(HOLDS['d20'])}** (up to {_sd(HOLDS['d20_hi'])} at the dearest listing) → ${tot + HOLDS['d20']:,.0f}.",
         f"- **D24 inline GIM8108-36 at shoulder roll: in the total** (owner OK 2026-11-27). Against the belt roll it costs "
         f"**{_sd(HOLDS['inline'])}** (up to {_sd(HOLDS['inline_hi'])} at the dearest listing).",
         "- Prices: [M1_BENCH_ARM.md](../M1_BENCH_ARM.md) (GIM8108-8 ≈ $115 ESTIMATE; GIM8108-36 $240 with the SDC303 driver).", "",
         "| Category | Subtotal |", "|---|---|"] + [f"| {c} | ${v:,.0f} |" for c, v in cats.items()] + [
         "", "| Category | Item | Qty | Unit $ | Line $ | Source |", "|---|---|---|---|---|---|"]
    L += [f"| {c} | {n} | {q} | {u:,.2f} | {q * u:,.0f} | {s} |" for c, n, q, u, s in I]
    return "\n".join(L), tot


if __name__ == "__main__":
    md, tot = report()
    (DOCS / "BOM_v4.md").write_text(md)
    I = items()
    (DOCS / "cost_down.md").write_text(cost_down_report(I, tot))
    sv = sorted(savings(I), key=lambda r: -r["s"])
    (DOCS / "bom.json").write_text(json.dumps({"total": tot, "with_d20": tot + HOLDS["d20"], "with_d20_inline": tot + HOLDS["d20"],
                                               "base_timer": {"selected": K.BASE_TIMER, "status": "pending owner decision (W15b U20; default board provisionally)",
                                                              "totals": timer_totals()},
                                               "w15b_w14b": {"before": W15_BEFORE, "before_d20": W15_BEFORE_D20},
                                               "options": {n[:60]: q * u * (1 + CONTINGENCY) for n, q, u, _, _ in options()},
                                               "subsystems": {k: round(v * (1 + CONTINGENCY), 2) for k, v in subsystems(I).items()},
                                               "d29": {"before": D29_BEFORE, "before_d20": D29_BEFORE_D20,
                                                       "rows": [{"n": i, "item": t, "status": st, "landed": round(s * (1 + CONTINGENCY), 2)} for i, t, st, _, s in d29_rows(I)[0]]},
                                               "savings_top10": [{"item": r["t"], "landed": round(r["s"] * (1 + CONTINGENCY), 2), "gate_neutral": not r.get("flag")} for r in sv]},
                                              indent=1))
    print(md[:1500])
