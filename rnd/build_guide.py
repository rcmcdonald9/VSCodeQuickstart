"""Build guide v4 (order of operations + hands-on time estimate).  python -m rnd.build_guide

Wave-1 design (2026-11-27): D24 bolted arm cartridges with an Oldham drive (the D15 socket steps go from the arm; KX sockets
stay on the head), D19 one-piece shaft-pulleys in capped seats, sw/bringup.py for bring-up. W10 (2026-11-27): the inline roll,
the lift as a bolted stage (lift v2), D24 v2 flange ears with nuts, a 6061 Oldham motor hub (the printed ASA tongue was
rejected, W8b), steel crush sleeves at the C1 / inline-roll bolts (W8c), D18b casters + ballast (W8d).
"""
from __future__ import annotations

import json
from pathlib import Path

DOCS = Path(__file__).resolve().parent.parent / "docs"

# (step, what, count, minutes each) — ESTIMATEs for a first build by one person with the parts printed
STEPS = [
    ("1 Print", "Print the parts in print_plan.md order: the bolted cans and rear caps first in ASA (they let you bench-test motors early)", 1, 0),
    ("2 Coupons", "Print + test the bolted-flange coupon (4 ears, M4 × 12 + washers into captive nuts, centring lip), the Oldham coupon (6061 motor hub on the GIM "
     "output against the bought D50 disc, 1.5 × the 11 N·m peak) and the GIM bolt-circle coupon; measure the printed C1 / rear-mount stacks and order or face the "
     "crush sleeves to print − 0.05 mm; adjust the flange / bolt circles in rnd/parts before mass printing", 1, 60),
    ("3 KX sockets", "D15 screwed socket, head pan + tilt only (D24: the arm has none; lift v2 bolts): 6 dowels, 6 coil springs, collar, retainer (3 × M3 into inserts); "
     "3 balls into the male face with retaining compound", 2, 8),
    ("4a Bolted cartridges", "D24 v2: 2 × M3 heat-set inserts in the sleeve lugs; motor into the sleeve from the rear, rear cap (3 × DIN 7984 M3 × 8 to the motor + "
     "2 × M3 × 10 into the lugs); 6061 Oldham motor hub on the output (4 × DIN 7984 M3 × 6, Loctite 243); label + CAN ID (pitch, yaw, elbow × 2 + lift)", 7, 9),
    ("4b Wrist cartridges", "D24: DM-J4310 into its can, bolted can flange and output flange (inserts + 6 × M3 each); label + CAN ID", 2, 9),
    ("4c KX + servo cartridges", "Wheels ×4: motor into its can (3 screws), torque pin, KX adapter (4 screws); STS3215 sleeves ×6", 10, 6),
    ("5 Belt stages", "D13 v3 + D19: 6801s into the eccentric bushings, keyed Ø12 shaft through wall A, keyed aluminium pulley, wall B; keyed Oldham clamp hub on the "
     "shaft end + disc; the one-piece shaft-pulley with its bearings into the capped U-slots, caps on (2 × M4 / M3); belt on, tension with the bushings; "
     "cartridge nose into the counterbore, 4 ear screws M4 × 12 + washers into captive / side-fed nuts (pitch: the rear pair M4 × 16 from the pulley side "
     "through the outer wall); 6 arm stages + the lift (lift v2)", 7, 15),
    ("5b Clock springs", "Wind the 6 ribbon cassettes per arm (D14 ribbon) and fit them at pitch, roll, yaw, elbow, wrist pitch, wrist roll", 2, 30),
    ("6 Arms", "Bond the CF tubes into the sockets and the 6061 Ø30 core tube into the elbow shaft (structural epoxy, cure overnight); upper bone + elbow "
     "cassette (in from below, seat caps), forearm + clamp (2 × M5), wrist + hand + gripper, roll/yaw module; yaw flange tie: 6 × M3 in Ø4.6 crush "
     "sleeves from the tube top (W8c)", 2, 95),
    ("7 Shoulders", "Pitch modules: bracket body, one-piece pitch shaft, C1 tie-rods (4 studs, crank, 8 Ø7 crush sleeves, clamp ring, nuts; W8c), outer wall "
     "(3 × M4), pitch hub, balancer (die spring, cord, pulley A at the 40–60 mm slot), V-band ferrule", 2, 55),
    ("7b Inline roll", "GIM8108-36 into the roll link: output adapter on 3 dowels bonded with Loctite 638, rear plate + 6 Ø6 crush sleeves + 2 steel stop bushes, "
     "6 × M4 × 25 csk + 2 × M4 × 25 SHCS into the motor; 61806 on the stub (DIN 471), outer race bonded with Loctite 660 at mid-travel; clevis-arm M5 in "
     "8 Ø8 crush sleeves (W8c)", 2, 35),
    ("8 Mast + torso + head", "Ratchet strip segments, lift drive + belt, carriage (8 × 625), torso halves around the carriage, head bracket + pan/tilt + shell", 1, 150),
    ("9 Base", "4 wheel modules (pods, shafts, hubs, wheels), deck quarters, skirt, coupling, battery bay, lidar, electronics; 4 D18b casters (rods, lock plate, "
     "spring, 24 V solenoid) into their guides, 2 ballast stacks bolted and dropped into their cradles (W8d)", 1, 150),
    ("10 Wiring", "Pre-made daisy-chain harness segments (JST-GH CAN/TTL, VH power) per module; power distribution board; detach plugs", 1, 120),
    ("10b Sensors (D9)", "Joint-side encoder magnets + boards (9), fingertip tactile pads (4), palm ToF (2), cliff ToF (4), 3 RP2040 CAN sensor nodes", 1, 160),
    ("11 Bring-up", "sw/bringup.py per arm (enumerate, drive limits to flash, e-stop + watchdog, hard-stop zeroing, 10 % jog, gravity hold); "
     "balancer K from the fit, lift pawl test, regen clamp test", 1, 120),
]
# The loop-40 steps the wave-1 design replaced (for the before/after line): 13 KX sockets × 8 min, 21 cartridges × 6 min
OLD = dict(sockets=(13, 8), cartridges=(21, 6), arm=6.05)


def hours():
    return sum(c * m for _, _, c, m in STEPS) / 60


def arm_hours():
    """One arm, parts printed: 3 bolted GIM6010 cartridges + the inline roll + the wrist DM + 2 servo sleeves, 3 belt stages, clock
    springs, the arm, its shoulder, its harness + bring-up (ESTIMATE). No KX socket on the arm (D24)."""
    m = dict((s.split()[0], mm) for s, _, _, mm in STEPS)
    return (3 * m["4a"] + m["4b"] + 2 * m["4c"] + 3 * m["5"] + m["5b"] + m["6"] + m["7"] + m["7b"] + 30 + 30) / 60


def main():
    h, ah = hours(), arm_hours()
    pp = json.loads((DOCS / "rnd" / "print_plan.json").read_text())
    prev = json.loads((DOCS / "rnd" / "build.json").read_text()) if (DOCS / "rnd" / "build.json").exists() else {}
    L = ["# Pip v4: build guide (generated by `python -m rnd.build_guide`)", "",
         f"**First build:** about **{h:.0f} h hands-on** (ESTIMATE), plus about **{pp['hours_p2s']:.0f} h of unattended P2S printing** "
         f"({pp['kg']:.1f} kg PETG/ASA). Reconfiguring later: an arm cartridge swap is 4 ear screws (M4 + nuts) with the belt on (~3 min); an arm comes off "
         "with one V-band clamp.",
         f"**One arm** (the first milestone build): about **{ah:.1f} h** hands-on"
         + (f" ({prev['arm_hours']:.2f} → {ah:.2f} h, {60 * (ah - prev['arm_hours']):+.0f} min, since the last guide)" if prev.get("arm_hours") else "")
         + (f", plus ≈ {pp['arm_hours_p2s']:.0f} h of printing." if pp.get("arm_hours_p2s") else "."), "",
         "Parts: [BOM](rnd/BOM_v4.md) · M1 kit: [M1_BENCH_ARM.md](M1_BENCH_ARM.md) · prints: [print plan](rnd/print_plan.md) · wiring: "
         "[electrical](rnd/electrical.md) · checks: [assembly](rnd/assembly.md), [sim](rnd/sim.md)", "",
         "| Step | What | Count | Min each | Hours |", "|---|---|---|---|---|"]
    L += [f"| {s} | {w} | {c} | {m} | {c * m / 60:.1f} |" for s, w, c, m in STEPS]
    so, co = OLD["sockets"], OLD["cartridges"]
    new_sc = sum(c * m for s, _, c, m in STEPS if s.split()[0] in ("3", "4a", "4b", "4c"))
    L += [f"| | **Total hands-on** | | | **{h:.1f}** |", "",
          f"**Wave 1 (D24 bolted cartridges, D19):** sockets + cartridges {(so[0] * so[1] + co[0] * co[1]) / 60:.1f} → {new_sc / 60:.1f} h for the robot; "
          f"one arm {OLD['arm']:.2f} → {ah:.2f} h ({60 * (ah - OLD['arm']):+.0f} min). The 5 D15 sockets per arm go (−40 min); the bolted "
          "cartridges' inserts and Oldham hub take about half of that back. D19's one-piece shaft-pulleys replace the "
          "split pulleys (no keyed joint to fit) at about the same time, the seat-cap screws included.", "",
          "**Against R10** (first build ≤ 4 h): **not met.**",
          "- **Loop 10 cut the estimate from 25 h to ~19 h:**",
          "    - print-in-place KX sockets with pins and spring embedded at print pauses (no retainer, no screws) — *retired in loop 40 (D15): the screwed socket takes ≈ 8 min; D24 then removed the sockets from the arm*",
          "    - 7 instead of 14 screws per cartridge",
          "    - eccentric-bushing belt tensioning",
          "    - pre-made harness segments",
          "- **A 21-actuator robot won't reach 4 h.** Precedent: a 6-servo LeRobot SO-101 arm takes a few hours.",
          "- **Proposed target split (owner decision):** first build of one arm ≤ 4 h; whole robot ≤ 15 h; any module swap ≤ 5 min.",
          "",
          "## Safety notes",
          "- 24 V Li-ion: use the specified BMS (≥ 30 A), fuse every branch, and charge on a non-flammable surface.",
          "- The arms can pinch. Torque limits are set in the drives (bring-up step 2, saved to flash) and bounded again by the safety "
          "supervisor. The bolted arm cartridges have **no mechanical slip backstop** (the KX seats slipped at ≈ 1.16× peak); only the head "
          "sockets still do.",
          "- Test the lift pawl, the e-stop and the watchdog (sw/bringup.py step 3) before any powered arm motion; bench work with the catch "
          "cradle ≤ 50 mm under the arm."]
    (DOCS / "BUILD_v4.md").write_text("\n".join(L) + "\n")
    (DOCS / "rnd" / "build.json").write_text(json.dumps({"hours": h, "arm_hours": ah}))
    print("\n".join(L[:5]))


if __name__ == "__main__":
    main()
