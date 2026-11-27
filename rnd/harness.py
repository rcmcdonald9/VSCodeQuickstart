"""Cable routing and harness kit (queue item 1, loop 32).  python -m rnd.harness

The question: can the arm bundle cross every joint over its full range without a slip ring? Two methods are checked per
joint (all cable data ESTIMATE until the datasheets are in hand):

1. **Torsion through the hollow shaft.** The bundle runs along the axis and twists. A torsion-rated robot cable allows
   about ±360°/m (TORSION_DEG_PER_M), so the free axial length L must be ≥ (range / 2) / 360 m.
2. **Clock-spring spiral** (as in a car's steering-wheel clock spring). The bundle lies in a flat spiral in an annular
   cavity r_in … r_out around the axis. Rotation winds or unwinds it, so the cable bends but never twists. The capacity
   for a spiral of length S was taken as Δθ = S·(1/r_in − 1/r_out) rad; loop 47 replaces it with a packing model that
   counts the ribbon thickness (below). It needs
   r_in ≥ the bend radius of the stiffest cable (10 × OD for continuous flex) and a cavity of the bundle's height.

The joint ranges come from the twin (rnd/twin.py JR). The free lengths and the cavity sizes are read off the CAD
(module dimensions; ESTIMATE ± 10 mm).
"""
from __future__ import annotations

import json
import math
from pathlib import Path

DOCS = Path(__file__).resolve().parent.parent / "docs" / "rnd"
TORSION_DEG_PER_M = 360.0          # torsion-rated robot cable class (ESTIMATE: check the datasheet)
BEND_K = 10.0                      # continuous-flex bend radius = 10 × OD (ESTIMATE)

# cable catalogue: (count, OD mm). D14 (loop 40): past the shoulder the arm bundle is 24 V + CAN + USB only; the elbow
# sensor node makes 12 V (buck) and drives the TTL servo bus locally, so 12 V/TTL no longer cross pitch, roll and yaw.
# Neon review #2 finding 10: the 24 V pair was 2 × 20 AWG silicone (OD 1.8); now 2 × 16 AWG high-flex with a thin
# fluoropolymer (ETFE/FEP) wall, OD ≤ 1.9 mm (ESTIMATE: M22759/16-16 class is ≈ 1.7 mm with 19 strands; the high-flex
# stranding adds ≈ 0.2 mm). ≤ 2.2 mm keeps the ribbon thickness (the USB sets it) and so every capacity below; width 10.0
# ≤ the 10.3 mm the 11.3 mm cassettes allow. Fluoropolymer also slides (μ ≈ 0.1 against the PTFE-taped cassette faces)
# where silicone sticks and slips.
CABLES = {"24 V power pair (2 × 16 AWG high-flex ETFE/FEP, OD 1.9 ESTIMATE; was 20 AWG silicone)": (2, 1.9), "2 CAN twisted pairs (26 AWG; A14: two channels per arm)": (4, 1.0),
          "USB 2.0 slim 4-core high-flex, shielded 90 Ω pair (OD 2.2; D12)": (1, 2.2)}
ELBOW_ON = {"12 V + GND + TTL (servos, 24 AWG)": (3, 1.2)}      # from the elbow node onward
STOCK_USB_OD = 3.8                  # the camera's stock USB cable (what loop 32 first checked)
# Loop 47 (Astra #2b finding 15): an annulus is not a clock spring. The ribbon has a radial thickness t (its thickest cable
# + the carrier tape), so wound tight on r_in it makes n_in turns and lying loose on r_out only n_out turns:
#   S = 2π·n_in·(r_in + n_in·t/2)   and   S = 2π·n_out·(r_out − n_out·t/2);   capacity Δθ = 360°·(n_in − n_out).
# The zero-thickness formula S(1/r_in − 1/r_out) lets all the cable sit on either radius and overstates the capacity.
TAPE_T = 0.1                        # PET/Kapton carrier tape (mm, ESTIMATE)
PACKING = 0.85                      # a real ribbon does not lie perfectly close-wound: t_eff = t / PACKING (ESTIMATE; measure on C11)


def spiral_turns(S_mm, ri, ro, t):
    """(n_in, n_out, capacity deg) for a ribbon of length S and radial thickness t (mm) in the annulus ri..ro."""
    n_in = (-ri + math.sqrt(ri * ri + t * S_mm / math.pi)) / t
    disc = ro * ro - t * S_mm / math.pi
    if disc < 0:
        return n_in, float("nan"), 0.0                      # does not fit even loose
    n_out = (ro - math.sqrt(disc)) / t
    return n_in, n_out, max(360.0 * (n_in - n_out), 0.0)


def best_length(ri, ro, t):
    """Ribbon length (mm) with the largest capacity for this annulus (≈ 50 % fill, the clock-spring optimum)."""
    return max((spiral_turns(S, ri, ro, t)[2], S) for S in range(50, 4000, 10))[::-1]


def ro_needed(ri, t, need_deg, ro0):
    """Smallest outer radius (0.5 mm steps) whose best ribbon length reaches need_deg."""
    ro = ro0
    while best_length(ri, ro, t)[1] < need_deg and ro < ro0 + 40:
        ro += 0.5
    return ro


def _roll_yaw():
    from .parts import roll
    return roll.ROLL_SPR, roll.YAW_SPR, roll.CAV_H


_RS, _YS, CAV_H = _roll_yaw()


def _pitch_wrist():
    from .params import BASELINE
    from .parts import shoulder
    return shoulder.PITCH_SPR, round(2 * BASELINE.arm.wrist_roll_range_deg)


_PS, WR_RANGE = _pitch_wrist()
# per joint crossed by the arm bundle: range (deg, from twin JR), free axial length (m) for torsion, spiral cavity
# (r_in, r_out, mm), spiral length S (m), and where. Roll and yaw cavities are real CAD (rnd/parts/roll.py, loop 40).
JOINTS = [
    # pitch: bundle from the torso e-tray through the pitch shaft bore (Ø30) to the detach plug on the ferrule
    ("shoulder pitch", 260, 0.20, _PS, 0.80, "CAD (2026-11-27): clock spring outboard of the outer bracket wall, around the pitch hub (r 25) inside a static ring on the wall; inner end into the hub bore through a side window (the balancer crank closes the tube's inboard end)"),
    ("shoulder roll", 115, 0.10, _RS, 0.25, "CAD: clock-spring cassette on the +X end of the roll shaft, outboard of the clevis arm (roll_spring_hub / static ring on a bar to the roll-link boss)"),
    ("upper-arm yaw", 300, 0.0, _YS, 0.75, "CAD: clock-spring cassette between the 40T yaw pulley's hub and the bottom yaw plate (yaw ±150°, hard stops ±155°)"),
    ("elbow", 150, 0.13, (22, 50), 0.60, "elbow housing, +Y side, between cassette plate B (y 26) and the +Y wall (y 53)"),
    ("wrist pitch", 220, 0.06, (16, 30), 0.40, "wrist can end, around the DM-J4310 (Ø57 can: little room)"),
    ("wrist roll", WR_RANGE, 0.03, (14, 24), 0.35, "palm, around the STS3215 wrist-roll servo (range ±120° since 2026-11-27: params.wrist_roll_range_deg)"),
]


def bundle(name):
    """Cables crossing a joint. Past the wrist only 12 V/TTL (gripper servo) + I²C (tactile, ToF) + the camera FFC go on."""
    if name.startswith("wrist"):        # D12: the camera is MIPI-CSI on a 0.3 mm FFC through the wrist (bend r ~2 mm); bridge on the forearm
        return {"12 V + GND + TTL (servos, 24 AWG)": (3, 1.2), "I²C 4-core (tactile + ToF)": (4, 0.8), "CSI FFC 15-way (camera)": (1, 0.3)}
    b = dict(CABLES)
    if name == "elbow":
        b.update(ELBOW_ON)
    return b


def analyse():
    rows = []
    for name, rng, L, (ri, ro), S, where in JOINTS:
        b = bundle(name)
        od_max = max(od for _, od in b.values())
        r_bend = BEND_K * od_max
        torsion_ok = L > 0 and (rng / 2) <= TORSION_DEG_PER_M * L
        ri_eff = max(ri, r_bend)
        cap_thin = math.degrees(S * (1 / (ri_eff / 1000) - 1 / (ro / 1000))) if ro > ri_eff else 0.0
        t = od_max + TAPE_T
        n_in, n_out, cap_ideal = spiral_turns(S * 1000, ri_eff, ro, t)
        cap = spiral_turns(S * 1000, ri_eff, ro, t / PACKING)[2] if ro > ri_eff else 0.0
        width = sum(n * od for n, od in b.values()) if not name.startswith("wrist") else None
        build = S * 1000 * od_max / (2 * math.pi * ri_eff)                 # radial build when wound tight on r_in (mm)
        fits = build <= (ro - ri_eff) * 0.6 and (width is None or name not in ("shoulder roll", "upper-arm yaw") or width <= CAV_H - 1.0)
        spiral_ok = cap >= rng * 1.2 and fits
        od_nousb = max(max(od for _, od in b.values()), STOCK_USB_OD)
        ri2 = max(ri, BEND_K * od_nousb)
        cap2 = spiral_turns(S * 1000, ri2, ro, (od_nousb + TAPE_T) / PACKING)[2] if ro > ri2 else 0.0
        rows.append(dict(t=t, n_in=n_in, n_out=n_out, cap_ideal=cap_ideal, cap_thin=cap_thin,
                         joint=name, range=rng, L=L, torsion_ok=torsion_ok, need_L=rng / 2 / TORSION_DEG_PER_M,
                         r_bend=r_bend, cavity=(ri, ro), S=S, spiral_cap=cap, spiral_ok=spiral_ok, build=build, width=width,
                         cap_nousb=cap2, ok_nousb=cap2 >= rng * 1.2, where=where,
                         area=sum(n * math.pi * od ** 2 / 4 for n, od in b.values())))
    return rows


def loop47(rows):
    """Clock-spring capacity with ribbon thickness (Astra #2b finding 15) for the two cassettes, and the fix as numbers."""
    L = ["**Loop 47: clock-spring capacity with the ribbon thickness (Astra #2b finding 15).** "
         f"The ribbon is as thick as its thickest cable plus the tape (Ø2.2 USB + {TAPE_T} mm = 2.3 mm); a real ribbon packs at ≈ {PACKING:.0%} (ESTIMATE), "
         "so the table's capacity uses t / 0.85. Astra's ideal close-wound check (400 mm, 2.2 mm, 22.5–40 mm) gives "
         f"{spiral_turns(400, 22.5, 40, 2.2)[2]:.0f}° for yaw; with the 2.3 mm ribbon {spiral_turns(400, 22.5, 40, 2.3)[2]:.0f}°, packed "
         f"{spiral_turns(400, 22.5, 40, 2.3 / PACKING)[2]:.0f}°: **yaw is short of the 360° it needs (±150° × 1.2).**", "",
         "| Cassette | Range × 1.2 | Now (S, packed) | Best S in this annulus | Fix A: longer ribbon only | Fix B: larger r_out (best S) | Fix C: thinner ribbon |", "|---|---|---|---|---|---|---|"]
    for r in rows:
        if r["joint"] not in ("shoulder roll", "upper-arm yaw") and r["spiral_cap"] >= r["range"] * 1.2:
            continue
        ri, ro = max(r["cavity"][0], r["r_bend"]), r["cavity"][1]
        need = r["range"] * 1.2
        te = r["t"] / PACKING
        S_best, cap_best = best_length(ri, ro, te)
        ro_req = ro_needed(ri, te, need * 1.1, ro)                           # 10 % reserve on top of the 1.2 factor
        S2, cap2 = best_length(ri, ro_req, te)
        # thinnest ribbon that reaches the need in the current annulus (a flat-flex USB, power pair on edge)
        t_req = next((t for t in [x / 10 for x in range(23, 4, -1)] if best_length(ri, ro, t / PACKING)[1] >= need * 1.1), None)
        L.append(f"| {r['joint']} | {need:.0f}° | {r['S']:.2f} m: {r['spiral_cap']:.0f}° {'✅' if r['spiral_cap'] >= need else '❌'} | "
                 f"{S_best / 1000:.2f} m: {cap_best:.0f}° {'✅' if cap_best >= need else '❌'} (≈ 50 % fill) | "
                 f"{'S = ' + f'{S_best / 1000:.2f} m' if cap_best >= need * 1.1 else 'not enough on its own'} | "
                 f"r_out {ro:.1f} → **{ro_req:.1f} mm**, S {S2 / 1000:.2f} m: {cap2:.0f}° | "
                 f"{'t ≤ ' + f'{t_req:.1f} mm' if t_req else '—'} (packed, current annulus, best S) |")
    w = rows[1]["width"]
    L += ["",
          f"- **Yaw fix (recommended): r_out 40 → {ro_needed(22.5, 2.3 / PACKING, 396, 40):.1f} mm with the ribbon lengthened to the best S** "
          "(Fix B); or keep r_out and go to a thinner ribbon (Fix C, needs a flat-flex USB: the 20 AWG pair alone is 1.8 mm). The ribbon length "
          "costs 24 V drop: 20 AWG ≈ 0.033 Ω/m per conductor (Astra 16), so each extra 0.4 m of loop is ≈ 0.3 V at 11 A.",
          f"- **Roll** has margin with the current 0.25 m (packed {rows[1]['spiral_cap']:.0f}° vs 138°), but its r_in 22 mm equals the USB's 10×OD bend radius: zero reserve (Astra 15). "
          "The D24 inline-roll redesign should keep r_in ≥ 24 mm.",
          f"- **Ribbon width (new):** A14 puts **two** CAN pairs through the shoulder, so the ribbon is {w:.1f} mm wide, not 7.8 mm; the cassettes are {CAV_H} mm high "
          f"(width ≤ {CAV_H - 1:.1f} mm). Fix: cavity height ≥ {w + 1.5:.1f} mm, or 28 AWG CAN pairs (OD 0.8 → {w - 0.8:.1f} mm wide, cavity ≥ {w - 0.8 + 1.5:.1f} mm).",
          f"- **Wrist roll:** ±180° needed 432°; the 14–24 mm palm annulus gives {rows[5]['spiral_cap']:.0f}° packed with its 1.3 mm bundle. "
          f"**2026-11-27: wrist roll limited to ±{WR_RANGE // 2}° ({WR_RANGE * 1.2:.0f}° needed)** instead of r_out 24 → 27.5: the palm axis is taken by "
          "the servo horn, dog and spindle bearings, so a bigger cassette means an Ø74 ring around the bearing skirt or +17 mm of hand length; the "
          "parallel-jaw gripper is 180°-symmetric, so ±90° already covers every jaw orientation (params.wrist_roll_range_deg).",
          "- C11 (10k reversals) must measure the real packing factor and the capacity at both stops; these are ideal-geometry numbers."]
    return L


# ---- Neon review #2 finding 10: 24 V drop and heating along the arm, USB length, the lift umbilical -----------------
OHM_PER_M = {24: 0.0842, 22: 0.0530, 20: 0.0333, 18: 0.0210, 16: 0.0132, 14: 0.00829, 12: 0.00521}   # copper at 20 °C (AWG tables)
HOT = 1 + 0.00393 * 40              # conductor at 60 °C
K_AIR, GAP = 0.026, 0.5e-3          # W/(m·K); air gap between the ribbon edges and each cassette face (m, ESTIMATE)
T_AMB, T_MAX = 40.0, 60.0           # °C inside the arm covers; ribbon limit (PETG cassette, Tg ≈ 80 °C, 20 K margin)
USB_MAX = 5.0                       # m per USB 2.0 cable segment (USB 2.0 spec: 26 ns cable delay ≈ 5 m)
R_XT60 = 0.0005                     # Ω per contact pair (ESTIMATE)


def _seg_lengths():
    S = {j[0]: j[4] for j in JOINTS}
    # (segment, one-way length m, loads downstream). The pitch motor is fed on the torso by its own lead; the elbow node
    # makes 12 V for the two STS3215 (D14), so their current is a 24 V load at the node.
    return [("torso e-tray → pitch spiral entry", 0.45, ("shoulder_roll", "upper_yaw", "elbow", "wrist", "servos")),
            ("pitch clock spring", S["shoulder pitch"], ("shoulder_roll", "upper_yaw", "elbow", "wrist", "servos")),
            ("detach plug (XT60) → roll motor", 0.10, ("shoulder_roll", "upper_yaw", "elbow", "wrist", "servos")),
            ("roll clock spring + roll link", S["shoulder roll"] + 0.10, ("upper_yaw", "elbow", "wrist", "servos")),
            ("yaw clock spring + upper arm → elbow motor / node", S["upper-arm yaw"] + 0.40, ("elbow", "wrist", "servos")),
            ("elbow clock spring + forearm → DM-J4310", S["elbow"] + 0.35, ("wrist",))]


def arm_power():
    """Per-segment 24 V current (sustained and worst peak for the drop: the two farthest GIMs stalled), voltage drop and
    ribbon heating for the old 20 AWG pair and the 16 AWG one."""
    from . import electrical as el
    a = el.arm_currents(servos=True)
    run = {r[0]: r[2] for r in a["rows"]}
    stall = {r[0]: r[3] for r in a["rows"]}
    sus = {j: 0.5 * run[j] for j in run} | {"wrist": 0.5 * el.J4310["i_nom"], "servos": 0.3 * a["servos"]}
    pk = dict(run) | {"upper_yaw": max(stall["upper_yaw"], run["upper_yaw"]), "elbow": max(stall["elbow"], run["elbow"]),
                      "wrist": el.J4310["i_peak"] * 24 / el.V_MIN * 0.5, "servos": a["servos"]}
    t_rib = max(od for _, od in CABLES.values()) + TAPE_T
    G = 2 * K_AIR * (t_rib / 1000) / GAP                              # W/(m·K) per metre of ribbon, edges to the two faces (ESTIMATE)
    segs, out = _seg_lengths(), {}
    for awg in (20, 16):
        r = OHM_PER_M[awg] * HOT
        rows, d_sus, d_pk = [], 0.0, 0.0
        for name, Lm, loads in segs:
            i_s, i_p = sum(sus[x] for x in loads), sum(pk[x] for x in loads)
            d_sus += 2 * r * Lm * i_s
            d_pk += 2 * r * Lm * i_p
            q = 2 * r * i_s ** 2                                      # W/m, both conductors, sustained
            rows.append(dict(seg=name, L=Lm, i_s=i_s, i_p=i_p, q=q, dT=q / G, spiral="clock spring" in name, d_pk=d_pk, d_sus=d_sus))
        d_pk += 2 * R_XT60 * rows[2]["i_p"]
        out[awg] = dict(rows=rows, d_sus=d_sus, d_pk=d_pk, r=r)
    usb = sum(Lm for _, Lm, _ in segs)                                # bridge at the wrist end of the forearm → hub in the torso e-tray
    return dict(out=out, G=G, t_rib=t_rib, usb=usb, total=sum(Lm for _, Lm, _ in segs))


# Lift umbilical (finding 10: missing). The torso rides 0.70 m on the mast (params.upper.lift_min/max) and the lift motor
# rides the carriage (omega belt, 2 clamps); the battery, contactor and branch fuses are in the base. A vertical drag chain
# fixed at mid-stroke: chain = stroke/2 + π·R; cable = chain + the fixed runs at both ends.
CHAIN = dict(R=0.050, inner=(15.0, 30.0), base_run=0.35, torso_run=0.30, fill_max=0.6)
UMB = [  # (circuit, conductors, AWG or None, OD mm ESTIMATE, sustained A, peak A, fuse A in the base)
    ("left arm 24 V (switched)", 2, 14, 3.4, None, None, 20),
    ("right arm 24 V (switched)", 2, 14, 3.4, None, None, 20),
    ("lift motor + pawl solenoid 24 V (switched; motor on the carriage)", 2, 18, 2.0, None, None, 10),
    ("12 V DC-DC input (head servos; switched)", 2, 18, 2.0, None, None, 7.5),
    ("logic 24 V (always on: Pi 5 DC-DC, screen, Jetson reserve)", 2, 18, 2.0, None, None, 5),
    ("CAN A twisted pair (base node, lift, D11 screen)", 2, None, 1.0, 0, 0, None),
    ("e-stop loop, torso mushroom: ch1 pair + ch2 pair", 4, None, 1.0, 0, 0, None),
    ("heartbeat, AUX_OK, ESTOP_SENSE + GND (safety board in the base, Pi in the torso)", 4, None, 0.8, 0, 0, None),
    ("USB 2.0 (RPLIDAR C1 in the base → torso hub), shielded", 1, None, 3.8, 0, 0, None),
]


def umbilical():
    from .params import BASELINE as B
    from . import electrical as el
    br = {n: (c, p) for n, c, p, _ in el.branches()}
    arm = next(v for k, v in br.items() if k.startswith("Left arm"))
    cur = [arm, arm, br["Lift cartridge + pawl solenoid"], next(v for k, v in br.items() if k.startswith("12 V")),
           (sum(v[0] for k, v in br.items() if k.startswith(("5 V", "Reserved"))), sum(v[1] for k, v in br.items() if k.startswith(("5 V", "Reserved"))))]
    stroke = B.upper.lift_max - B.upper.lift_min
    chain = stroke / 2 + math.pi * CHAIN["R"]
    Lc = chain + CHAIN["base_run"] + CHAIN["torso_run"]
    rows = []
    for k, (name, n, awg, od, _, _, fuse) in enumerate(UMB):
        i_s, i_p = cur[k] if k < len(cur) else (0.0, 0.0)
        r = OHM_PER_M[awg] * HOT if awg else None
        rows.append(dict(name=name, n=n, awg=awg, od=od, i_s=i_s, i_p=i_p, fuse=fuse,
                         drop=2 * r * Lc * i_p if r else 0.0, q=r * i_s ** 2 if r else 0.0))
    area = sum(r["n"] * math.pi * r["od"] ** 2 / 4 for r in rows)
    od_max = max(r["od"] for r in rows)
    up_s = sum(r["i_s"] for r in rows if r["awg"])
    return dict(rows=rows, stroke=stroke, chain=chain, L=Lc, area=area, fill=area / (CHAIN["inner"][0] * CHAIN["inner"][1]),
                od_max=od_max, ok_R=CHAIN["R"] * 1000 >= 10 * od_max, up_s=up_s)


def f10_lines():
    ok = lambda b: "✅" if b else "❌"
    ap, um = arm_power(), umbilical()
    o20, o16 = ap["out"][20], ap["out"][16]
    L = ["## Neon review #2 finding 10: arm power wire, USB, lift umbilical", "",
         f"**Arm 24 V pair, verified in part.** The review's 0.216 Ω (6.5 m of 20 AWG) at 20–30 A puts the whole arm current through the whole length. Per segment "
         f"(sustained = every big joint at 50 % of rated power; peak = the two farthest GIMs stalled + roll at rated + wrist peak + servos stalled; conductors at 60 °C) "
         f"the drop at the DM-J4310 is **{o20['d_pk']:.1f} V with 20 AWG** (the review's 4.3–6.5 V range is right) and {o16['d_pk']:.1f} V with 16 AWG. It does **not** trip "
         f"a drive: 20 V − {o20['d_pk']:.1f} V = {20 - o20['d_pk']:.1f} V against the GIM's 12 V minimum; it costs speed headroom. The heating claim is right for the "
         f"wound spiral: ribbon edges conduct to the two cassette faces across ≈ {GAP * 1e3:.1f} mm of air, G ≈ 2·k_air·t/gap = {ap['G']:.2f} W/(m·K) (ESTIMATE; C11 measures it).", "",
         "| Segment | Length (m) | Sustained / peak (A) | 20 AWG: W/m, ribbon ΔT | 16 AWG: W/m, ribbon ΔT | |", "|---|---|---|---|---|---|"]
    for a, b in zip(o20["rows"], o16["rows"]):
        lim = T_MAX - T_AMB
        L.append(f"| {a['seg']} | {a['L']:.2f} | {a['i_s']:.1f} / {a['i_p']:.1f} | {a['q']:.1f} W/m, {a['dT']:.0f} K | {b['q']:.1f} W/m, {b['dT']:.0f} K | "
                 f"{('in a cassette: ' + ok(b['dT'] <= lim)) if a['spiral'] else 'free run'} |")
    L += [f"| **Drop to the DM-J4310 (sustained / peak, incl. the XT60)** | {ap['total']:.2f} | | {o20['d_sus']:.2f} / **{o20['d_pk']:.2f} V** | {o16['d_sus']:.2f} / **{o16['d_pk']:.2f} V** | |", "",
          f"Limit: ribbon ≤ {T_MAX:.0f} °C at {T_AMB:.0f} °C inside the covers (ΔT ≤ {T_MAX - T_AMB:.0f} K). **Fix: 2 × 16 AWG high-flex ETFE/FEP (OD ≤ 1.9 mm)**; it keeps the "
          "ribbon thickness (the USB's 2.2 mm sets it), so the capacities above do not change. The sustained case is the fuse-sizing one; the duty-cycle "
          "budget (budget.md) puts the arms near 1 A each, so the cassettes normally run cool. C11 (10k reversals) runs **with 12 A on the pair and the wrist camera "
          "streaming 60 fps**, logging the ribbon temperature and USB re-enumerations.", "",
          f"**USB 2.0 HS, verified in part.** Bridge (forearm, wrist end) → hub in the torso e-tray: **{ap['usb']:.2f} m**, under the {USB_MAX:.0f} m segment limit "
          f"({USB_MAX - ap['usb']:.1f} m spare), so length is not the problem; the review is right that the detach plug had **no USB pins** (electrical.md's 6-pin pogo block) while "
          "the harness kit listed a USB-C there. Fix, without moving the camera off USB (CAN cannot carry video; Ethernet adds magnetics and a camera change):",
          "- a real shielded USB cable (90 Ω twisted data pair inside its own jacket) in the ribbon, never loose cores; on the ribbon edge away from the power pair, the CAN pairs between;",
          "- the detach plug becomes a 10-pin pogo block with D+ / D− on adjacent pins flanked by GND pins (≈ 5 mm of pin: short against the 480 Mbps edge), "
          "the shield on GND at the hub end only; the powered USB hub moves to the torso e-tray (so the arm run is one segment);",
          "- acceptance in C11 as above; fallback if it fails: a USB 2.0 redriver at the torso side of the plug (ESTIMATE ≈ $5), then a manual latching connector (M8 / GX12 class).", "",
          f"**Lift umbilical (missing before), verified.** The torso rides {um['stroke']:.2f} m on the mast; the battery, contactor and branch fuses are in the base, the Pi in the torso. "
          f"A vertical drag chain fixed at mid-stroke: chain {um['chain']:.2f} m (stroke / 2 + π × R {CHAIN['R'] * 1e3:.0f} mm), cable {um['L']:.2f} m one way with the fixed runs.", "",
          "| Circuit | Conductors | Wire | Sustained / peak (A) | Base fuse | Drop at peak (V) | W/m per conductor (sustained) |", "|---|---|---|---|---|---|---|"]
    L += [f"| {r['name']} | {r['n']} | {str(r['awg']) + ' AWG silicone' if r['awg'] else f'Ø{r['od']:.1f} signal'} | "
          f"{(f'{r['i_s']:.1f} / {r['i_p']:.1f}') if r['awg'] else '–'} | {r['fuse'] or '–'} | {f'{r['drop']:.2f}' if r['awg'] else '–'} | {f'{r['q']:.2f}' if r['awg'] else '–'} |"
          for r in um["rows"]]
    L += ["", f"- Chain: inner {CHAIN['inner'][0]:.0f} × {CHAIN['inner'][1]:.0f} mm, bend R {CHAIN['R'] * 1e3:.0f} mm ≥ 10 × the thickest cable (Ø{um['od_max']:.1f}) {ok(um['ok_R'])}; "
          f"fill {um['fill']:.0%} of the cross-section (≤ {CHAIN['fill_max']:.0%}) {ok(um['fill'] <= CHAIN['fill_max'])}. Chain-rated (torsion-free, fine-strand) cable; the two arm pairs and the lift pair are separate circuits "
          "so each branch fuse still protects its own wire.",
          f"- Coupling at the mast foot: the upper's sustained load is {um['up_s']:.1f} A, over one XT60 (30 A); now 3 × XT60 (left arm, right arm, lift + 12 V) with the "
          "always-on logic feed on two pogo pins (electrical.md *Connectors*).",
          "- Space claim for the CAD owner (rnd/parts/upper.py, base.py): the chain's moving loop needs ≈ 2R + chain height ≈ 120 mm beside the mast over the lower half of the stroke, "
          "fixed end at mast mid-height, moving end on the carriage.", ""]
    return L


def main():
    rows = analyse()
    n_ok = sum(r["torsion_ok"] or r["spiral_ok"] for r in rows)
    n_ok2 = sum(r["torsion_ok"] or r["ok_nousb"] for r in rows)        # with the stock USB cable
    L = ["# Cable routing across the joints (generated by `python -m rnd.harness`)", "", __doc__.split("\n\n", 1)[1].strip(), "",
         f"**Joints the arm bundle can cross: {n_ok}/{len(rows)} with D12** (CSI flat-flex through the wrist + slim Ø2.2 USB from the forearm); "
         f"{n_ok2}/{len(rows)} with the camera's stock Ø3.8 USB cable.", "",
         "| Joint | Range | Torsion: free length / needed | Spiral cavity r_in–r_out (mm) | Bend r (10×OD) | Ribbon t | Spiral S: turns tight / loose | Capacity: zero-thickness / close-wound / packed 85 % (need) | Ribbon build / width | Method | Stock USB |",
         "|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        m = "torsion ✅" if r["torsion_ok"] else ("clock spring ✅" if r["spiral_ok"] else "❌")
        L.append(f"| {r['joint']} | {r['range']}° | {r['L'] * 1000:.0f} / {r['need_L'] * 1000:.0f} mm {'✅' if r['torsion_ok'] else '❌'} | "
                 f"{r['cavity'][0]}–{r['cavity'][1]} | {r['r_bend']:.0f} mm | {r['t']:.1f} mm | {r['S']:.2f} m: {r['n_in']:.2f} / {r['n_out']:.2f} | "
                 f"{r['cap_thin']:.0f}° / {r['cap_ideal']:.0f}° / **{r['spiral_cap']:.0f}°** ({r['range'] * 1.2:.0f}°) | "
                 f"{r['build']:.1f} mm / {'—' if r['width'] is None else f"{r['width']:.1f} mm"} | {m} | "
                 f"{'✅' if r['torsion_ok'] or r['ok_nousb'] else '❌'} {r['cap_nousb']:.0f}° |")
    L += [""] + loop47(rows)
    L += ["", "**Findings (loop 32):**",
          "- **No joint can rely on torsion through its bore.** The free axial lengths are 30–200 mm, and the ranges need 160–500 mm at ±360°/m. "
          "The old electrical note ('cables through the hollow shafts, no slip rings') needs **clock-spring spirals** at each joint.",
          "- **The camera's stock Ø3.8 USB cable** needs a 38 mm bend radius and fits through none of the wrist joints. **D12:**",
          "  - a MIPI-CSI camera on a thin FFC through both wrist joints",
          "  - a CSI→USB (UVC) bridge board on the forearm",
          "  - a slim 4-core high-flex USB cable (OD ≤ 2.2 mm) from there on",
          "- **Loop 40: roll and yaw cassettes are now real CAD** (external review finding 4). Yaw is back to **±150°** (loop 33 had cut it to ±70°, "
          "which cost side grasps 68 → 40 %):",
          "  - **yaw:** a 9.5 mm cavity between the split 40T pulley's hub (rotating inner wall, r 22.5) and the bottom yaw plate, static ring r 40 screwed to the plate; the ribbon's inner end enters the yaw-shaft bore and runs down the neck",
          "  - **roll:** a cassette on the +X end of the roll shaft, outboard of the clevis arm; the static ring hangs on a bar that runs inboard at z ≈ 0 (a direction the roll output never sweeps) to the roll-link boss",
          "  - **D14:** the elbow sensor node makes 12 V and drives the TTL servo bus, so only 24 V + CAN + USB (a 7.8 mm flat ribbon) cross pitch, roll and yaw",
          "- The ribbon: the round cables side by side on a PET/Kapton carrier tape, wound as in a car's clock spring (cable bends, never twists).",          "",
          "## Harness kit (pre-made, one per arm + torso + base)",
          "| Segment | Contents | Length (ESTIMATE) | Ends |", "|---|---|---|---|",
          "| Torso e-tray → shoulder detach plug | 24 V pair (2 × 16 AWG ETFE/FEP), 2 CAN pairs, shielded USB | 0.45 m + 0.80 m pitch spiral | XT60 + 10-pin pogo (CAN ×2, USB D+/D−, GND, ID) |",
          "| Detach plug → elbow sensor node | same | 0.10 m + roll 0.25 + link 0.10 + yaw 0.75 + upper arm 0.40 | daisy-chain JST-GH / VH at each cartridge |",
          "| Elbow node → hand | 24 V (DM-J4310), 12 V/TTL (servos), I²C, camera (FFC per option 1) | 0.60 m elbow spiral + 0.35 m + 2 × wrist spirals | JST-GH / FFC |",
          f"| **Lift umbilical: base → torso (new)** | 2 × arm pairs 14 AWG, lift + 12 V + logic pairs 18 AWG, CAN A, e-stop ×2, safety signals, USB (lidar) | {umbilical()['L']:.2f} m in a drag chain (R 50) | base fuse block / torso e-tray |",
          "| Mast: torso → head | 24 V (F 1 A), CAN A, TTL 3 | 0.75 m in a cable chain along the lift travel | JST-GH |",
          "| Base ↔ upper coupling | 3 × XT60 + 12-pin pogo (CAN A, e-stop loop, safety signals, logic 24 V) | 0.3 m | blind-mate |"] + [""] + f10_lines()
    (DOCS / "harness.md").write_text("\n".join(L))
    (DOCS / "harness.json").write_text(json.dumps({"ok": n_ok, "n": len(rows), "ok_nousb": n_ok2}))
    print("\n".join(L[4:15]))


if __name__ == "__main__":
    main()
