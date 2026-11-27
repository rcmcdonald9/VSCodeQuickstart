"""Motor thermal check: continuous task duty and worst-case hold times (confidence campaign).  python -m rnd.thermal

Model (per motor, first order). Copper loss scales with torque², so the steady winding rise is
ΔT_ss = ΔT_rated,mount · (τ / τ_rated)². τ_rated is the maker's continuous torque, taken to give ΔT_RATED (60 K) with the
motor **open in still air** (ESTIMATE; the maker's test mounting is not published). Heating follows ΔT(t) = ΔT_ss · (1 − e^(−t/TAU)).

**Mounting (Fable review M7, 2026-11-27).** The arm motors do not sit in open air: each is inside a closed 3 mm ASA sleeve
with a printed rear cap and a plastic Oldham coupling, so there is no metal conduction path, and the R17 covers go over
that. The old model added a flat 10 K for this. Now a thermal network: winding → case (a fixed share F_WC of the open-air
rise) → case → ambient, where the open motor has R_open = 1 / (h_o · A_motor) and the sleeved motor
R_sleeve = 1 / (h_i · A_motor) + t / (k_ASA · A_sleeve) + 1 / (h_o · A_sleeve) (trapped air inside, the ASA wall, still air
outside). The ratio K = R_sleeve / R_open scales the case-to-ambient part of the rise:
ΔT_rated,mount = ΔT_RATED · (F_WC + (1 − F_WC) · K). Mitigation row: an **aluminium rear cap** bolted to the motor's rear face
(thermal pad) with a finned face open to the room, a parallel path R_cap = R_contact + 1 / (h_o · A_fins).

Duty cases:
1. **Task day.** The MuJoCo fetch cycle repeated back to back with no rest, in the Monte Carlo worst corner: arms ×1.25,
   joint friction 0.6 N·m, motors at 75 % torque constant (so the same torque needs more current: τ_rated × 0.75), 0.6 kg can.
2. **Worst static hold.** The worst workspace pose with the 1 kg payload, held indefinitely. The result is the time to
   reach the limit.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

DOCS = Path(__file__).resolve().parent.parent / "docs" / "rnd"
AMBIENT, T_LIMIT = 30.0, 100.0      # °C: warm room; winding/driver limit (ESTIMATE: GIM/SDC104 class over-temp cut-out ~100 °C)
DT_RATED = 60.0                     # K rise at rated torque, motor open in still air (ESTIMATE)
TAU = 600.0                         # s thermal time constant, 60–80 mm outrunner class (ESTIMATE; longer when sleeved: ignored)
WORST = dict(derate=0.75, mass=1.25, joint_friction=0.6, finger_mu=0.5, calib=np.zeros(6))
# Thermal network constants (all ESTIMATE; the M1 bench logs the drive temperature and a sleeve thermistor to replace them)
H_OUT, H_IN = 12.0, 5.0             # W/m²K: still air outside (convection + radiation); trapped air in a narrow closed gap
K_ASA, T_WALL = 0.17, 0.003         # W/mK, m: ASA sleeve wall
F_WC = 0.30                         # share of the open-air rise that is winding → case (stator to housing)
R_CONTACT = 0.5                     # K/W motor rear face → aluminium cap through a thermal pad, 3 screws
FIN_AREA = 3.0                      # finned cap face = 3 × its disc area open to the room
COVER_DT = 5.0                      # K: local air under the R17 covers above the room (ESTIMATE; the old model's flat 10 K covered sleeve + covers)
# Motor envelope (actuators.yaml / drawings); the sleeve is Ø+6.6 mm and 30 mm longer (cartridge_m: Ø86.6 for the GIM6010)
ENVELOPE = {"GIM6010-8": (0.080, 0.040), "GIM8108-8": (0.096, 0.045), "GIM8108-36": (0.096, 0.057), "DM-J4310-2EC_V1.1": (0.056, 0.046)}
ACT_OF = {"pitch": "shoulder_pitch", "roll": "shoulder_roll", "yaw": "upper_yaw", "elbow": "elbow", "wpitch": "wrist_pitch"}
CAN_LIMIT = {"PETG": 55.0, "ASA": 75.0}   # long-term wall limits (HDT 70 / 94 °C less creep margin)


def network(name="GIM6010-8"):
    """Open, sleeved and sleeved + aluminium cap: case-to-ambient resistance ratios to the open motor, and the share of
    the sleeved rise that the sleeve wall sees."""
    D, L = ENVELOPE.get(name, ENVELOPE["GIM6010-8"])
    A_m = math.pi * D * L + 2 * math.pi * (D / 2) ** 2
    Ds, Ls = D + 0.0066, L + 0.030
    A_s = math.pi * Ds * Ls + 2 * math.pi * (Ds / 2) ** 2
    R_open = 1 / (H_OUT * A_m)
    R_wall = T_WALL / (K_ASA * A_s) + 1 / (H_OUT * A_s)
    R_sl = 1 / (H_IN * A_m) + R_wall
    R_cap = R_CONTACT + 1 / (H_OUT * FIN_AREA * math.pi * (Ds / 2) ** 2)
    R_sc = 1 / (1 / R_sl + 1 / R_cap)
    R_wc = F_WC / (1 - F_WC) * R_open
    return dict(K=R_sl / R_open, K_cap=R_sc / R_open, R_open=R_open, R_sleeve=R_sl, R_cap=R_cap,
                wall_frac=R_wall / (R_wc + R_sl))


def dt_rated(k):
    return DT_RATED * (F_WC + (1 - F_WC) * k)


def hold_time(dt_ss):
    room = T_LIMIT - AMBIENT - COVER_DT
    return math.inf if dt_ss <= room else -TAU * math.log(1 - room / dt_ss)


def main():
    from . import sim
    from .params import BASELINE
    from .spring import workspace_margins
    L = sim.limits()
    r = sim.run(video=False, can_mass=0.6, perturb=WORST)
    rows, worst = [], dict(old=0.0, sleeve=0.0, cap=0.0, wall=0.0)
    for n, f in r["force"].items():
        if n == "a_fing_a_r":
            continue
        j = n[2:].replace("_r", "")
        act_name = BASELINE.arm.joints[ACT_OF[j]][0] if j in ACT_OF else ""
        nw = network(act_name or "GIM6010-8")
        rated = L[n][0] * WORST["derate"]
        rms = math.sqrt(np.mean(np.square(f)))
        x2 = (rms / rated) ** 2
        t_old = AMBIENT + 10.0 + DT_RATED * x2
        sleeved = j in ACT_OF
        if sleeved:
            t_sl, t_cap = AMBIENT + COVER_DT + dt_rated(nw["K"]) * x2, AMBIENT + COVER_DT + dt_rated(nw["K_cap"]) * x2
            wall = AMBIENT + COVER_DT + dt_rated(nw["K"]) * x2 * nw["wall_frac"]
        else:                                        # wrist roll servo, lift and wheels: not in an arm sleeve
            t_sl = t_cap = t_old
            wall = AMBIENT
        worst = {k: max(worst[k], v) for k, v in dict(old=t_old, sleeve=t_sl, cap=t_cap, wall=wall).items()}
        rows.append((j, act_name, rms, rated, t_old, t_sl, t_cap, wall, sleeved))
    nw = network("GIM6010-8")
    hold = []
    for j, m in workspace_margins().items():
        k = network(m["act"])
        x2 = (m["max"] / m["rated"]) ** 2
        dts, dtc = dt_rated(k["K"]) * x2, dt_rated(k["K_cap"]) * x2
        hold.append((j, m["max"], m["rated"], AMBIENT + COVER_DT + dts, hold_time(dts), AMBIENT + COVER_DT + dtc, hold_time(dtc),
                     m["rated"] * math.sqrt((T_LIMIT - AMBIENT - COVER_DT) / dt_rated(k["K"]))))
    ok = lambda b: "✅" if b else "❌"
    fmt = lambda t: "unlimited ✅" if math.isinf(t) else f"{t / 60:.0f} min"
    Lr = ["# Motor thermal check (generated by `python -m rnd.thermal`)", "", __doc__.split("\n\n", 1)[1].strip(), "",
          f"Ambient {AMBIENT:.0f} °C, limit {T_LIMIT:.0f} °C, ΔT at rated (open, still air) {DT_RATED:.0f} K, τ {TAU:.0f} s; h_o {H_OUT:.0f} / h_i {H_IN:.0f} W/m²K, "
          f"F_WC {F_WC:.0%}, +{COVER_DT:.0f} K local air under the covers, ASA {K_ASA} W/mK × {T_WALL * 1000:.0f} mm (all ESTIMATE until the M1 bench logs a real motor's temperature).", "",
          "| Mounting (GIM6010-8 cartridge) | Case-to-ambient vs open (K) | ΔT at rated | Continuous torque for the 100 °C limit |", "|---|---|---|---|",
          f"| Open in still air + 10 K (the old model) | 1.00 | {DT_RATED + 10:.0f} K | {math.sqrt((T_LIMIT - AMBIENT - 10) / DT_RATED):.2f} × rated |",
          f"| **Closed ASA sleeve, printed cap, plastic coupling (as drawn)** | **{nw['K']:.2f}** | **{dt_rated(nw['K']):.0f} K** | **{math.sqrt((T_LIMIT - AMBIENT - COVER_DT) / dt_rated(nw['K'])):.2f} × rated** |",
          f"| Sleeve + aluminium rear cap, finned face to the room | {nw['K_cap']:.2f} | {dt_rated(nw['K_cap']):.0f} K | {min(1.0, math.sqrt((T_LIMIT - AMBIENT - COVER_DT) / dt_rated(nw['K_cap']))):.2f} × rated |", "",
          "## 1. Task day: fetch cycle back to back, Monte Carlo worst corner", "",
          "| Actuator | RMS torque/force | Rated (derated ×0.75) | Winding, old model (open + 10 K) | **Winding, closed sleeve** | Winding, sleeve + Al cap | Sleeve wall (ASA ≤ 75 °C) |",
          "|---|---|---|---|---|---|---|"]
    Lr += [f"| {j}{f' ({a})' if a else ''} | {rms:.2f} | {rt:.2f} | {to:.0f} °C | **{ts:.0f} °C** {ok(ts <= T_LIMIT)} | {tc:.0f} °C {ok(tc <= T_LIMIT)} | "
           f"{(f'{w:.0f} °C ' + ok(w <= CAN_LIMIT['ASA'])) if sl else 'not in a sleeve (old model)'} |" for j, a, rms, rt, to, ts, tc, w, sl in rows]
    Lr += ["", f"**Hottest winding on a non-stop task day: {worst['sleeve']:.0f} °C in the closed sleeve** (the old open + 10 K model said {worst['old']:.0f} °C; "
           f"with an aluminium rear cap {worst['cap']:.0f} °C; limit {T_LIMIT:.0f} °C). The sleeve wall runs cooler than the old model said "
           f"({worst['wall']:.0f} °C: the trapped air inside holds the heat back), so **the limit is the winding, not the ASA**. "
           "**Day-1 actions (Fable M7):** a 10 k NTC on every GIM sleeve plus the drive's own temperature in the bring-up log; an aluminium "
           "rear cap on the first cartridge (Step 1 of the M1 kit) to measure the difference; vents in the R17 covers over each cap.", "",
           "## 2. Worst static hold (worst workspace pose, 1 kg payload, nominal motors)", "",
           "| Joint | Worst static torque (N·m) | Rated (N·m) | Steady winding, sleeve | Time to the limit | With Al cap | Time to the limit | Continuous torque in the sleeve (N·m) |",
           "|---|---|---|---|---|---|---|---|"]
    Lr += [f"| {j} | {mx:.1f} | {rt:.1f} | {Ts:.0f} °C | {fmt(ts)} | {Tc:.0f} °C | {fmt(tc)} | {cont:.1f} |" for j, mx, rt, Ts, ts, Tc, tc, cont in hold]
    lim = [h[4] for h in hold if not math.isinf(h[4])]
    Lr += ["", ("Every joint can hold its worst pose indefinitely." if not lim else
                f"**Some joints can't hold their worst pose forever in the closed sleeve (shortest {min(lim) / 60:.0f} min).** The planner must "
                "derate long holds at full reach (the driver's thermal model already does this). Normal reaching is far below the worst pose.")]
    (DOCS / "thermal.md").write_text("\n".join(Lr) + "\n")
    (DOCS / "thermal.json").write_text(json.dumps({"task_max_c": worst["sleeve"], "task_max_c_open_model": worst["old"], "task_max_c_al_cap": worst["cap"],
                                                   "hold_min_s": min(lim) if lim else None, "can_max_c": worst["wall"],
                                                   "k_sleeve": nw["K"], "k_cap": nw["K_cap"], "dt_rated_sleeve": dt_rated(nw["K"])}))
    print("\n".join(Lr))


if __name__ == "__main__":
    main()
