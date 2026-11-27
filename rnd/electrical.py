"""Electrical architecture: 24 V distribution, fusing, e-stop chain, CAN/TTL buses, connectors.  python -m rnd.electrical

Bus currents are estimated per motor as (mechanical power + copper loss) / bus voltage.
Motor constants are ESTIMATEs back-calculated from the specs: GIM6010-8 5 N·m at 10.5 A continuous → Kt ≈ 0.48 N·m/A
at the output, and a phase resistance of ~0.35 Ω.
"""
from __future__ import annotations

import math
from pathlib import Path

from .params import BASELINE, G, act

DOCS = Path(__file__).resolve().parent.parent / "docs" / "rnd"
V_MIN, V_NOM = 20.0, 22.2           # 6S: 3.33 V/cell cut-off, 3.7 V/cell nominal
GIM = dict(kt=5.0 / 10.5, r=0.35)   # ESTIMATE
J4310 = dict(i_nom=2.5, i_peak=7.5)  # actuators.yaml


def gim_bus_current(torque, speed_rad_s):
    i = torque / GIM["kt"]
    return (torque * speed_rad_s + i * i * GIM["r"]) / V_MIN


def branches(d=BASELINE):
    g = act("GIM6010-8")
    rated_run = gim_bus_current(g["rated"], g["speed"])        # rated torque at rated speed
    peak_stall = gim_bus_current(g["peak"], 0.0)                # peak torque, stalled (worst copper loss)
    arm_cont = 4 * rated_run * 0.5 + J4310["i_nom"] * 0.5       # duty ~50 % of rated on the big joints (twin v1: ≤ 37 % RMS)
    arm_peak = 2 * peak_stall + 2 * rated_run + J4310["i_peak"] * 24 / V_MIN * 0.5
    base_cont, base_peak = 4 * gim_bus_current(0.8, 12.6), 2 * peak_stall + 2 * rated_run     # carpet cruise; threshold climb
    lift_cont, lift_peak = gim_bus_current(2.4, 12.6), peak_stall
    servo12 = 6 * 2.7 * 12 / V_MIN * 0.3                        # 6 × STS3215, 30 % of stall on average
    logic = (5 * 5 + 15) / V_MIN                                 # Pi 5 (5 V 5 A max) + cameras / lidar / CAN adapters
    rows = [("Base: 4 wheel cartridges", base_cont, base_peak, 20), ("Left arm: 4 GIM + DM-J4310", arm_cont, arm_peak, 15),
            ("Right arm: 4 GIM + DM-J4310", arm_cont, arm_peak, 15), ("Lift cartridge + pawl solenoid", lift_cont, lift_peak, 10),
            ("12 V DC-DC → 6 × STS3215 (wrists, grippers, head)", servo12, 6 * 2.7 * 12 / V_MIN * 0.6, 7.5),
            ("5 V DC-DC → Pi 5, cameras, lidar, CAN/TTL adapters", logic, logic * 1.3, 5),
            ("Reserved: 24 → 19 V 3 A DC-DC → Jetson-class AI computer (D10, AI phase)", 15 / V_MIN, 25 / V_MIN, 3)]
    return rows


# ---------------------------------------------------------------------------------------------------------------------
# A5 v2 (Astra review #2b findings 17 + 18): regen clamp sized by energy and pulse rating, switched pre-charge,
# category-0 drop protection, watchdog timing budget.
# ---------------------------------------------------------------------------------------------------------------------
PAYLOAD_20LB = 4.536                 # kg, R2c (owner: 20 lb carried two-handed) -> 2.27 kg per hand
D20 = {"shoulder_pitch": ("GIM8108-8", 3.0), "elbow": ("GIM8108-8", 3.0)}   # D20: GIM8108-8 at pitch + elbow, both 3:1
V_FULL = 25.2                        # 6S full pack (4.20 V/cell); also the bench PSU setting
V_DRIVE_MAX = 28.0                   # GIM6010 manual: 12-28 V (actuators.yaml); design limit for every driver on the bus
KV_OUT = {"GIM6010-8": 420 / 24,     # rpm/V at the output: 420 rpm max at 24 V (actuators.yaml, SteadyWin manual)
          "GIM8108-8": 6.67}         # rpm/V at the output (openelab listing; aifitlab gives 6.04: the faster one is the worst case)
J_ROTOR = {"GIM6010-8": 6e-5, "GIM8108-8": 1.5e-4}   # kg·m² rotor inertia at the motor shaft: ESTIMATE (outer rotor ring, Ø60/Ø81 class)
MOTOR_CU = {"GIM6010-8": (5.0 / 10.5, 0.35),        # output Kt (N·m/A), phase R (Ω): ESTIMATE (as GIM above)
            "GIM8108-8": (7.5 / 7.0, 0.25)}         # 7.5 N·m rated at 7 A nominal (listing); R ESTIMATE
COVER_KG = 0.5                       # D23 joint covers per arm (ESTIMATE, D23)
INERTIA_MARGIN = 1.15                # distributed-body inertia on top of the point masses (ESTIMATE)
# Clamp resistor: Vishay Dale RH-50 aluminium-housed wirewound, 50 W on its reference heatsink, short-time overload
# 5 × rated power for 5 s (Vishay RH datasheet; Mouser / eeworld copies). Two 0.56 Ω in parallel.
RH50 = dict(p_rated=50.0, overload=5.0, t_overload=5.0, price=15.0)   # price: RS £59.90 / 5 pcs (RH050 family) ≈ $15 each
CLAMP = dict(v_on=26.5, v_tol=0.01, hyst=0.4, n_res=2, r_each=0.56, adiabatic=0.25,
             mosfet="2 × Infineon IRFB7530PbF (60 V, 2.0 mΩ max, 195 A package limit; one per resistor)")
N_DRIVERS = {"robot": 15, "M1": 5}   # 13 GIM + 2 DM-J4310; M1: 4 GIM + 1 DM-J4310
C_BULK, C_DRIVER = 4700e-6, 470e-6   # the bus capacitor + per-driver input capacitance (ESTIMATE, not published)
I_IDLE_DRIVER = 0.04                 # A per driver at 24 V, logic only (ESTIMATE)
# Contactor: TE Kilovac ALEV100 (24 V coil, 100 A, 1 Form A aux contact 2 A @ 30 VDC). LEV100 family: release 10 ms,
# operate 25 ms max (DigiKey / Xonelec listings of LEV100A5ANG; ALEV100 datasheet 2025-02). LEV100A5ANG ≈ $92 (Master Electronics).
CONTACTOR = dict(name="TE Kilovac ALEV100 (24 V coil, 1 Form A aux)", t_release=0.010, t_operate=0.025, price=110.0)
# Watchdog: SN74LVC1G123 retriggerable monostable on its own 3.3 V LDO, t_w ≈ R·C (TI SLVA720); retriggered by the
# heartbeat edge the Pi emits only after a complete healthy 500 Hz cycle (sw/safety.py heartbeat_ok()).
WATCHDOG = dict(t_nom=0.008, tol=0.20, loop_hz=500.0)
R_PRE, T_PRE_TIMEOUT = 3.3, 1.0      # switched pre-charge: RH-50 3.3 Ω through a 10 A / 30 VDC relay; 1 s timeout
V_UVLO = 10.0                        # driver under-voltage lockout: ESTIMATE (GIM range starts at 12 V)


def _arm_inertias(d=BASELINE, payload_hand=PAYLOAD_20LB / 2):
    """Diagonal inertia (kg·m²) about pitch, roll, yaw, elbow, arm stretched in each axis's worst direction (point masses)."""
    a = d.arm
    m = lambda j: act(D20.get(j, a.joints[j])[0])["mass"]
    lu, lf, lh = a.l_upper, a.l_fore, a.l_hand
    hand = act("STS3215-C018")["mass"] * 2 + a.gripper_mech_mass + a.wrist_cam_mass
    distal = [(a.bone_mass_fore + COVER_KG / 2, lf / 2), (m("wrist_pitch"), lf), (hand, lf + 0.08), (payload_hand, lf + lh)]
    proximal = [(a.yaw_stage_mass, a.yaw_drop), (m("upper_yaw"), a.yaw_drop + 0.04), (a.bone_mass_upper + COVER_KG / 2, (a.yaw_drop + lu) / 2),
                (m("elbow"), lu)]
    I_sh = sum(mm * r * r for mm, r in proximal) + sum(mm * (lu + r) ** 2 for mm, r in distal)
    I_el = sum(mm * r * r for mm, r in distal)
    k = INERTIA_MARGIN
    return {"shoulder_pitch": k * I_sh, "shoulder_roll": k * I_sh, "upper_yaw": k * I_el, "elbow": k * I_el}


def regen_budget(d=BASELINE, payload=PAYLOAD_20LB, arms=2, lift=True):
    """Worst-case regenerated energy and peak bus power: both arms (all four belt joints) + the lift, each decelerated at the
    drive's peak torque from the highest speed it can reach (no-load speed at a full pack, or the speed reachable within
    half the joint range at peak torque, whichever is lower). Friction ignored (conservative); copper loss subtracted from
    the peak power only (ESTIMATE constants)."""
    from .twin import JR
    rng = {"shoulder_pitch": JR["pitch"], "shoulder_roll": JR["roll"], "upper_yaw": JR["yaw"], "elbow": JR["elbow"]}
    I = _arm_inertias(d, payload / 2)
    rows = []
    for j in ("shoulder_pitch", "shoulder_roll", "upper_yaw", "elbow"):
        name, belt = D20.get(j, d.arm.joints[j])
        g = act(name)
        n = 8 * belt                                          # planetary 8:1 × belt
        Ij = I[j] + J_ROTOR[name] * n * n                     # + reflected rotor
        tau = g["peak"] * belt
        w_nl = KV_OUT[name] * V_FULL * 2 * math.pi / 60 / belt
        half = math.radians(rng[j][1] - rng[j][0]) / 2
        w = min(w_nl, math.sqrt(2 * tau * half / Ij))
        E = 0.5 * Ij * w * w
        kt, r = MOTOR_CU[name]
        i_ph = g["peak"] / kt
        P = max(tau * w - i_ph * i_ph * r, 0.0)
        rows.append((j, name, Ij, w, E, P, Ij * w / tau))
    E_arm, P_arm = sum(r[4] for r in rows), sum(r[5] for r in rows)
    out = dict(rows=rows, E_arm=E_arm, P_arm=P_arm, E=arms * E_arm, P=arms * P_arm, arms=arms, payload=payload)
    if lift:
        from .lift import moving_mass
        from .belts import pd
        g = act(d.upper.lift_motor)
        r = pd(20) / 2 / 1000
        M = moving_mass(d) + payload + 2 * sum(act(n)["mass"] - act(d.arm.joints[j][0])["mass"] for j, (n, _) in D20.items())
        M_eq = M + J_ROTOR[d.upper.lift_motor] * (8 / r) ** 2      # rotor seen at the belt (8:1 planetary)
        v = KV_OUT[d.upper.lift_motor] * V_FULL * 2 * math.pi / 60 * r
        F = g["peak"] / r
        decel = (F - M * G) / M_eq                               # braking a lowering torso: gravity works against the drive
        s = v * v / (2 * decel)
        E_l = 0.5 * M_eq * v * v + M * G * s
        kt, rr = MOTOR_CU[d.upper.lift_motor]
        P_l = F * v - (g["peak"] / kt) ** 2 * rr
        out.update(M_lift=M, M_eq=M_eq, v_lift=v, E_lift=E_l, P_lift=P_l, P_lower=M * G * v, P_lower_rated=M * G * act(d.upper.lift_motor)["speed"] * r)
        out["E"] += E_l
        out["P"] += P_l
    return out


def clamp_check(rb):
    c = CLAMP
    R = c["r_each"] / c["n_res"]
    v_lo = c["v_on"] * (1 - c["v_tol"])
    v_hi = c["v_on"] * (1 + c["v_tol"]) + c["hyst"]
    P_cap = v_lo ** 2 / R                                            # sink power at the lowest threshold
    e_rating = RH50["overload"] * RH50["p_rated"] * RH50["t_overload"]   # 1,250 J per resistor (5 × 50 W for 5 s)
    e_allow = c["adiabatic"] * e_rating * c["n_res"]
    return dict(R=R, v_lo=v_lo, v_hi=v_hi, I_pk=v_hi / R, P_cap=P_cap, e_rating=e_rating, e_allow=e_allow,
                ok_P=P_cap >= rb["P"], ok_E=rb["E"] <= e_allow, ok_V=v_hi + 0.5 <= V_DRIVE_MAX, ok_off=v_lo - c["hyst"] > V_FULL,
                cont=c["n_res"] * RH50["p_rated"])


def c_bus(which="robot"):
    return C_BULK + N_DRIVERS[which] * C_DRIVER


def stop_timing(which="robot"):
    """Hardware path with the Pi hung (worst case): heartbeat lost → monostable times out → coil MOSFET off → contactor opens →
    aux contact opens → chopper forced on (bus discharge) → bus below driver UVLO → phase current gone."""
    w = WATCHDOG
    C = c_bus(which)
    R = CLAMP["r_each"] / CLAMP["n_res"]
    t_cycle = 1 / w["loop_hz"]
    t_detect = t_cycle + w["t_nom"] * (1 + w["tol"])          # the fault can hit just after a heartbeat edge
    t_dis = R * C * math.log(V_FULL / V_UVLO)
    t_dis_passive = C * (V_FULL - V_UVLO) / (N_DRIVERS[which] * I_IDLE_DRIVER)   # idle draw only (no forced discharge)
    rows = [(f"heartbeat lost → watchdog output low (one {t_cycle * 1e3:.0f} ms cycle + T_wd {w['t_nom'] * 1e3:.0f} ms +{w['tol']:.0%})", t_detect),
            ("coil MOSFET off → contactor main contacts open (release time, datasheet)", CONTACTOR["t_release"]),
            ("aux contact opens → chopper gate forced on", 0.0002),
            (f"bus {V_FULL:.1f} → {V_UVLO:.0f} V (driver UVLO, ESTIMATE) through the clamp resistors ({R:.2f} Ω × {C * 1e3:.1f} mF)", t_dis),
            ("phase current decays once the drive stops switching (ESTIMATE)", 0.001)]
    total = sum(t for _, t in rows)
    return dict(rows=rows, total=total, t_dis_passive=t_dis_passive, C=C,
                t_button=CONTACTOR["t_release"] + 0.0002 + t_dis + 0.001, t_sw=2 * t_cycle)


def precharge(which="robot"):
    C = c_bus(which)
    tau = R_PRE * C
    return dict(C=C, tau=tau, t95=3 * tau, i_in=V_FULL / R_PRE, p_in=V_FULL ** 2 / R_PRE, E=0.5 * C * V_FULL ** 2,
                droop=N_DRIVERS[which] * I_IDLE_DRIVER * R_PRE, e_fault=V_FULL ** 2 / R_PRE * T_PRE_TIMEOUT,
                e_rating=RH50["overload"] * RH50["p_rated"] * RH50["t_overload"])


def drop_catch(d=BASELINE, payload_hand=1.0, gap=0.05):
    """Category-0 drop on the bench: with drive torque gone the arm falls about the pitch axis. Energy from horizontal with
    no catch vs. an adjustable padded catch set ≤ gap under the forearm/hand in every powered pose."""
    a = d.arm
    m = lambda j: act(D20.get(j, a.joints[j])[0])["mass"]
    hand = act("STS3215-C018")["mass"] * 2 + a.gripper_mech_mass + a.wrist_cam_mass
    parts = [(a.roll_stage_mass + m("shoulder_roll"), 0.0), (a.yaw_stage_mass + m("upper_yaw"), a.yaw_drop),
             (a.bone_mass_upper + COVER_KG / 2, (a.yaw_drop + a.l_upper) / 2), (m("elbow"), a.l_upper),
             (a.bone_mass_fore + COVER_KG / 2, a.l_upper + a.l_fore / 2), (m("wrist_pitch"), a.l_upper + a.l_fore),
             (hand + payload_hand, a.l_upper + a.l_fore + a.l_hand)]
    M = sum(mm for mm, _ in parts)
    r_com = sum(mm * r for mm, r in parts) / M
    E_free = M * G * r_com - a.spring_k_nm              # horizontal → hanging, minus the balancer's work K·(1 − cos 90°)
    return dict(M=M, r_com=r_com, E_free=E_free, E_catch=M * G * gap, gap=gap, F_static=M * G, wll=10 * M * G)


def a5v2_section(d=BASELINE):
    rb = regen_budget(d)
    cc = clamp_check(rb)
    m1 = regen_budget(d, payload=2.0, arms=1, lift=False)
    st, st1 = stop_timing("robot"), stop_timing("M1")
    pc = precharge("robot")
    dc_m1, dc_r = drop_catch(d, 1.0), drop_catch(d, PAYLOAD_20LB / 2)
    ok = lambda b: "✅" if b else "❌"
    names = {"shoulder_pitch": "shoulder pitch", "shoulder_roll": "shoulder roll", "upper_yaw": "upper-arm yaw", "elbow": "elbow"}
    L = ["## A5 v2 (Astra review #2b, findings 17 + 18)", "",
         "### Regen clamp sized by energy and pulse rating",
         f"Worst case: **both arms + the lift decelerating at once, at the drives' peak torque, from the highest speed each joint can reach**, "
         f"with the {PAYLOAD_20LB:.2f} kg (20 lb) payload split over the two hands and the D20 motors (GIM8108-8 at pitch and elbow, 3:1). "
         "A joint's top speed is the lower of its no-load speed at a full pack (25.2 V) and the speed it can reach within half its range at peak torque "
         "(it has to stop in the other half). Inertia: point masses from params with the arm stretched in each axis's worst direction, +15 %, "
         "plus the reflected rotor (J_rotor ESTIMATE). Friction ignored for the energy (conservative); copper loss at peak current subtracted from the peak power.", "",
         "| Joint (each arm) | Motor | I + rotor (kg·m²) | Top speed (rad/s) | Energy (J) | Peak bus power (W) | Stop time (s) |", "|---|---|---|---|---|---|---|"]
    L += [f"| {names[j]} | {n} | {I:.2f} | {w:.1f} | {E:.0f} | {P:.0f} | {t:.2f} |" for j, n, I, w, E, P, t in rb["rows"]]
    L += [f"| lift (moving {rb['M_lift']:.1f} kg; {rb['M_eq']:.1f} kg with the rotor) | {d.upper.lift_motor} | – | {rb['v_lift']:.2f} m/s | {rb['E_lift']:.0f} | {rb['P_lift']:.0f} | – |",
          f"| **Robot total (2 arms + lift)** | | | | **{rb['E']:.0f} J** | **{rb['P']:.0f} W** | |",
          f"| M1 bench (one arm, 2 kg in hand, D20 motors) | | | | {m1['E']:.0f} J | {m1['P']:.0f} W | |", "",
          f"Continuous regen: lowering the torso at full speed returns {rb['P_lower']:.0f} W, at the rated {act(d.upper.lift_motor)['speed'] * 0.0159:.2f} m/s {rb['P_lower_rated']:.0f} W. "
          f"The clamp's continuous rating is {cc['cont']:.0f} W (two RH-50 on their reference heatsink), so **lowering is capped at the rated speed whenever the pack is full or the "
          "supply is a bench PSU** (planner rule; the clamp's thermal switch latches a fault if it is ignored).", "",
          f"**Part:** {CLAMP['n_res']} × **Vishay Dale RH-50, {CLAMP['r_each']} Ω** (aluminium-housed wirewound, 50 W on its heatsink; datasheet short-time overload "
          f"{RH50['overload']:.0f} × rated for {RH50['t_overload']:.0f} s = **{cc['e_rating']:.0f} J per resistor**) in parallel = {cc['R']:.2f} Ω, switched by {CLAMP['mosfet']}, "
          "a comparator with its own reference (threshold 26.5 V ± 1 %, 0.4 V hysteresis), a gate driver, 10 µF film capacitors at the MOSFETs (switching overshoot), "
          "and an 85 °C NC thermal switch on the resistor plate. **Mandatory on the M1 bench** (a bench PSU cannot sink current). The 33 V TVS stays only as an edge snubber: a TVS that does not conduct at 25.2 V "
          "clamps far above 28 V at current, so it cannot enforce the drive limit (Astra 17). On the bench, an ideal-diode stage stops regen reaching the PSU.", "",
          "| Check | Value | Limit | |", "|---|---|---|---|",
          f"| Sink power at the lowest threshold ({cc['v_lo']:.2f} V² / {cc['R']:.2f} Ω) | {cc['P_cap']:.0f} W | ≥ worst-case peak {rb['P']:.0f} W | {ok(cc['ok_P'])} |",
          f"| Pulse energy | {rb['E']:.0f} J | ≤ 25 % of the 5 s overload rating for a < 0.5 s pulse (adiabatic allowance, ESTIMATE): {cc['e_allow']:.0f} J | {ok(cc['ok_E'])} |",
          f"| Highest bus voltage while clamping (+1 % tol. + hysteresis) | {cc['v_hi']:.2f} V | ≤ {V_DRIVE_MAX:.0f} V − 0.5 V | {ok(cc['ok_V'])} |",
          f"| Clamp off at a full pack (lowest threshold − hysteresis) | {cc['v_lo'] - CLAMP['hyst']:.2f} V | > {V_FULL} V | {ok(cc['ok_off'])} |",
          f"| Peak current (MOSFETs, wiring) | {cc['I_pk']:.0f} A ({cc['I_pk'] / 2:.0f} A per MOSFET) | 195 A package limit each | ✅ |", "",
          "Check on the restrained rig (Astra 17): a dummy capacitive energy source first, then one axis; scope the bus **at the drive** for a simultaneous stop, a PSU trip and a contactor opening. "
          "Confirm the RH-50 pulse curve for a 20× / 0.3 s pulse with Vishay; the 5 s rating alone doesn't prove it.", "",
          "### Pre-charge, switched (not in parallel)",
          f"The loop-40 diagram drew 22 Ω permanently across the contactor: it would keep feeding ≈ 1 A into a stopped bus. Now a relay **K_PRE** (10 A / 30 VDC) in series "
          f"with a {R_PRE} Ω RH-50 closes only when both mushrooms are released, the watchdog is healthy and reset is pressed; the main contactor closes when the bus reaches "
          f"V_pack − 3 V; K_PRE then opens. Every stop opens K_PRE too (it hangs on the same safety chain), and it opens after {T_PRE_TIMEOUT:.0f} s if the bus never comes up (latched fault). "
          f"With {pc['C'] * 1e3:.1f} mF on the robot bus (4.7 mF + 15 drivers × 0.47 mF, ESTIMATE): τ = {pc['tau'] * 1e3:.0f} ms, 95 % in {pc['t95'] * 1e3:.0f} ms, inrush {pc['i_in']:.1f} A, "
          f"{pc['E']:.1f} J in the resistor; idle driver draw sags the bus only {pc['droop']:.1f} V. A shorted bus for the full timeout puts {pc['e_fault']:.0f} J in it (rating {pc['e_rating']:.0f} J). "
          "22 Ω would have left the bus ≈ 13 V short with the drivers' idle current.", "",
          "### Watchdog timing budget (SW2 < 50 ms)",
          f"The control loop runs at {WATCHDOG['loop_hz']:.0f} Hz and toggles the heartbeat line only after a complete healthy cycle. The watchdog is an SN74LVC1G123 "
          f"retriggerable monostable (t_w ≈ R·C, TI SLVA720) on its own 3.3 V LDO, set to {WATCHDOG['t_nom'] * 1e3:.0f} ms (four missed cycles, ±{WATCHDOG['tol']:.0%} with a C0G capacitor). "
          "Its output gates the MOSFET in series with the contactor coil, which also runs through both mushrooms' NC contacts. "
          "**The contactor alone isn't torque-off:** the drives keep running from the bus capacitors. The contactor's aux contact (main open) forces the clamp MOSFETs on, "
          "so the bus is discharged through the clamp resistors.", "",
          "| Step (robot, Pi hung) | Time (ms) |", "|---|---|"]
    L += [f"| {n} | {t * 1e3:.1f} |" for n, t in st["rows"]]
    L += [f"| **Total: heartbeat lost → no drive torque** | **{st['total'] * 1e3:.1f}** (M1 bench {st1['total'] * 1e3:.1f}) |", "",
          f"Mushroom pressed: {st['t_button'] * 1e3:.0f} ms. Without the forced discharge, the drivers' idle draw alone would take ≈ {st['t_dis_passive'] * 1e3:.0f} ms to bring the bus to UVLO: "
          "the old design could not meet 50 ms. The software path is faster when the Pi is alive: the supervisor latches, sends DISABLE to every drive the next cycle "
          f"(≤ {st['t_sw'] * 1e3:.0f} ms) and stops the heartbeat. Day-1 checks: the drives' UVLO and CAN-timeout behaviour, the ALEV100 release time with its coil suppression; "
          "measure button → contacts → current → motion with a supported dummy load, including a hung Pi.", "",
          "### Category-0 drop protection (bench)",
          f"Category 0 removes torque, so the arm falls (Astra 18). From horizontal, the M1 arm (D20 motors, 1 kg in hand, {dc_m1['M']:.1f} kg outboard of pitch, COM {dc_m1['r_com'] * 1000:.0f} mm) "
          f"releases **{dc_m1['E_free']:.0f} J** after the balancer's share; with a 10 lb half-basket it is {dc_r['E_free']:.0f} J. Rules for every powered bench test:",
          f"- **Mechanical catch:** an adjustable padded cradle (EVA foam on a bolted aluminium bar) set ≤ {dc_m1['gap'] * 1000:.0f} mm under the forearm or hand in every pose under test, "
          f"or a Dyneema tether from the stand with ≤ {dc_m1['gap'] * 1000:.0f} mm slack: drop energy ≤ {dc_m1['E_catch']:.1f} J. Rated ≥ {dc_m1['wll']:.0f} N (10 × the arm's weight).",
          "- Payload tray under the gripper; belt guards on; the stand bolted to the bench; operator outside the whole fall envelope (remote button).",
          "- On the robot (later): SS1 (controlled stop, then torque off) with a rated delay relay; the lift pawl holds the torso.", ""]
    return L


def can_load(motors, node_frames, hz, bits=130, rate=1e6):
    """Bus utilisation: 2 frames per motor per cycle (command + reply) + node frames (A14)."""
    return (2 * motors + node_frames) * bits * hz / rate


def report(d=BASELINE):
    rows = branches(d)
    cont = sum(r[1] for r in rows)
    peak = sum(r[2] for r in rows) * 0.6                          # not every branch peaks at once (diversity 0.6, ESTIMATE)
    L = ["# Electrical architecture (auto-generated by `python -m rnd.electrical`)", "",
         "## Power distribution (24 V class, 6S Li-ion)", "",
         "```",
         "6S 10 Ah pack (BMS ≥ 30 A cont.) ─ XT60 ─ MAIN FUSE 40 A ─┬─ LOGIC branch (always on): 5 V 5 A DC-DC → Pi 5, cameras, lidar, CAN/TTL adapters",
         "                                                         └─ E-STOP CONTACTOR (ALEV100, DC-rated; coil: mushroom NC ×2 in series → watchdog MOSFET; aux contact → feedback + discharge)",
         "                                                            └─ PRE-CHARGE branch around it: relay K_PRE (opens on every stop) + 3.3 Ω RH-50 (A5 v2: switched, never permanent)",
         "                                                              ├─ REGEN CLAMP / BUS DISCHARGE on the motor bus: 26.5 V ± 1 % comparator, 0.4 V hysteresis → 2 × IRFB7530 → 2 × 0.56 Ω RH-50 (A5 v2); forced on while the contactor is open",
         "                                                              ├─ 12 V 10 A DC-DC → STS3215 servos (A5: now downstream of the contactor)",
         "                                                              ├─ F 20 A → base: 4 wheel cartridges (CAN bus A)",
         "                                                              ├─ F 10 A → lift cartridge + pawl solenoid (CAN bus A)",
         "                                                              ├─ F 15 A → left arm via detach plug (CAN bus B, TTL bus 1)",
         "                                                              └─ F 15 A → right arm via detach plug (CAN bus C, TTL bus 2)",
         "LOGIC 24 V ─ F 1 A ─ D11 FACE SCREEN (ESP32-S3-Touch-LCD-4.3B, 7–36 V input; CAN A drop at the head; isolated DI senses the e-stop loop)",
         "DOCK PADS (skirt, 45°) ─ ideal-diode reverse-polarity guard ─ F 7.5 A ─ BMS charge port   (charger 25.2 V CC/CV, 5 A)",
         "```", "",
         "**Docking charge path (loop 27):**",
         "- The dock's output is off until the robot answers on the ID pad: the base sensor node pulls a coded load, the dock's ESP32-C3 checks it, then closes its MOSFET.",
         "- The robot may run while it charges (idle ≈ 33 W ≈ 1.4 A of the 5 A), so a stock charger could never see current fall to its end-of-charge level. The robot therefore tells the dock to stop at 4.15 V/cell over the ID line (from BMS telemetry).",
         "- **Check before buying** whether the 6S BMS is common-port (charge and load on one port) or split-port; the wiring above assumes common-port (ESTIMATE).", "",
         "| Branch | Worst sustained bus current (A), for fuse/wire sizing | Peak (A) | Fuse (A, slow-blow) |", "|---|---|---|---|"]
    L += [f"| {n} | {c:.1f} | {p:.1f} | {f} |" for n, c, p, f in rows]
    from .budget import power_budget
    _, _, avg_w, hours = power_budget(d)
    L += [f"| **Pack** | **{cont:.1f}** | **{peak:.0f}** (diversity 0.6) | main 40 |", "",
          f"Typical draw is far lower: the duty-cycle power budget averages **{avg_w:.0f} W ≈ {avg_w / V_NOM:.1f} A** (runtime {hours:.1f} h on 222 Wh).", "",
          f"**Battery requirement (new finding):** the pack's BMS must allow **≥ 30 A continuous and about {peak:.0f} A for ~1 s**. "
          "Cheap 6S packs often have 10–15 A BMS boards (components.yaml), which would trip on arm or threshold peaks. Buy a scooter or power-tool class pack, or build one around a 40 A BMS.",
          "",
          "## E-stop behaviour (A5, loop 40: external review finding 6)",
          "- **Two-channel mushrooms (torso back, base rear).** Channel 1 opens the contactor coil loop directly (hardware, no software in the path). "
          "Channel 2 tells the Pi and the sensor nodes, which command a controlled stop. Motor power drops; logic stays up to log and report.",
          "- **Stop category:** loop 39 had a *safe stop 1* with a 0.5 s capacitor hold-up. A capacitor delay is not a dependable timer, so the **default is now "
          "category 0** (power cut at once). The arms go limp; the balancer and belt friction slow the fall (≈ 0.3 s to the stop at the hanging pose; a 1 kg payload "
          "is released by the gripper's own back-drive). SS1 can return later with a rated delay relay.",
          "- **Everything that moves is downstream of the contactor:** loop 39 fed the 12 V servo converter from the logic branch, so the wrists, grippers and head "
          "kept power in an e-stop. It now sits after the contactor.",
          "- **Torso:** the lift pawl is spring-applied, so it drops in and the torso holds (A15 reviews the pawl itself).",
          "- Not a certified safety function. The bench arm runs inside a marked keep-out zone until SW2 is measured.",
          ""] + a5v2_section() + [
          "", "## Data buses",
          "| Bus | Members | Rate | Load at 500 Hz |", "|---|---|---|---|",
          f"| CAN A | 4 wheels + lift + base sensor node + D11 screen (≤ 50 Hz) | 1 Mbps, **250 Hz** | {can_load(5, 2, 250):.0%} |",
          f"| CAN B1 / C1 (one per arm) | shoulder pitch, roll, yaw + the encoder frame of the elbow node | 1 Mbps, 500 Hz | {can_load(3, 1, 500):.0%} |",
          f"| CAN B2 / C2 (one per arm) | elbow + DM-J4310 wrist pitch + the elbow node (tactile/ToF frame, TTL bridge for wrist roll + gripper, D14) | 1 Mbps, 500 Hz | {can_load(2, 3, 500):.0%} |",
          "| TTL 3 | head pan + tilt (torso node) | 1 Mbps half-duplex | light |",
          f"| *Loop 39 layout, for reference* | *one bus per arm: 5 motors + node at 500 Hz (with D14's bridge)* | | *{can_load(5, 4, 500):.0%}: too high (review finding 15)* |",
          "",
          "**A14 (loop 40):** each arm gets two CAN channels (≈ 45 % each, room for retries and jitter) and the base runs at 250 Hz. Five channels in all: "
          "five candleLight USB-CAN adapters (FYSETC UCAN class, gs_usb SocketCAN; loop 47 cost pass replaced the 2-channel SPI HAT). **M1 needs 2 channels** (one arm). "
          "Load model: 2 frames per motor per cycle (command + reply) and node frames, ≈ 130 bit per 8-byte frame with stuffing.",
          "| USB3 | head RGB-D (Gemini 2), 2 wrist cameras, lidar | – | Pi 5 has 2 × USB3; use a powered USB3 hub |", "",
          "## Connectors",
          "| Interface | Power | Signal |", "|---|---|---|",
          "| Arm detach plug (blind-mate, floats in a printed mount beside the V-band) | XT30 (24 V, 15 A branch) | 6-pin 2 mm pogo block: CAN1 H/L, CAN2 H/L, GND, ID strap (loop 40: D14 + A14; 12 V and TTL are made in the arm) |",
          "| Upper ↔ base coupling (under the cam lever) | XT60 | 6-pin pogo: CAN A H/L, e-stop loop ×2, 5 V sense, GND |",
          "| Battery slide-out | XT60 (keyed) | balance lead to the charger only |",
          "| Dock contacts (skirt at 45°, flush brass pads) | + at 60 mm, − at 100 mm, 60 × 12 mm pads, 5 A | ID pad at 40 mm → base sensor node (handshake, stop-charge command) |",
          "| Every joint cartridge | 2-pin JST-VH (24 V) or GH (12 V) | 4-pin JST-GH CAN/TTL daisy chain |", "",
          "**Cable routing (loop 32, rnd/harness.py):** torsion through the short shaft bores can't absorb the joint ranges (cable rated ~±360°/m). "
          "Every joint crossing uses a **clock-spring spiral** in an annular cavity around the joint axis. No slip rings. "
          "D12 keeps the stiffest cable at Ø2.2 mm: the wrist camera is MIPI-CSI on a flat-flex through the wrist, with a USB bridge on the forearm. "
          "Loop 40: yaw back to ±150° with a real clock-spring cassette; **D14:** the elbow node makes 12 V and drives the TTL servo bus, so only 24 V, CAN (two pairs, A14) "
          "and USB cross the shoulder joints. See [harness.md](harness.md)."]
    return "\n".join(L), cont, peak


if __name__ == "__main__":
    md, c, p = report()
    (DOCS / "electrical.md").write_text(md)
    print(md)
