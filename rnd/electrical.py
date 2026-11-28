"""Electrical architecture: 24 V distribution, fusing, e-stop chain, CAN/TTL buses, connectors.  python -m rnd.electrical

Bus currents are estimated per motor as (mechanical power + copper loss) / bus voltage.
Motor constants are ESTIMATEs back-calculated from the specs: GIM6010-8 5 N·m at 10.5 A continuous → Kt ≈ 0.48 N·m/A
at the output, and a phase resistance of ~0.35 Ω.
"""
from __future__ import annotations

import math
from pathlib import Path

from .params import BASELINE, SPECS, G, act
from .dock_sim import PADS_V2 as _PADS, PAD_W as DOCK_PAD_W

DOCK_PADS = {n: (z, h) for n, z, h in _PADS}      # Neon #2 F12 pad layout (rnd/dock_sim.py; the CAD track adopts it)

DOCS = Path(__file__).resolve().parent.parent / "docs" / "rnd"
V_MIN, V_NOM = 20.0, 22.2           # 6S: 3.33 V/cell cut-off, 3.7 V/cell nominal
GIM = dict(kt=5.0 / 10.5, r=0.35)   # ESTIMATE
J4310 = dict(i_nom=2.5, i_peak=7.5)  # actuators.yaml


def gim_bus_current(torque, speed_rad_s, name="GIM6010-8"):
    """Bus current (A) of one drive at the motor output: (mechanical power + copper loss) / V_MIN. Constants: MOTOR_CU."""
    kt, r = MOTOR_CU.get(name, (GIM["kt"], GIM["r"]))
    i = torque / kt
    return (torque * speed_rad_s + i * i * r) / V_MIN


STS_STALL_A, BUCK_EFF = 2.7, 0.90    # STS3215-C018 stall current at 12 V (actuators.yaml); D14 24 → 12 V buck efficiency (ESTIMATE)


def arm_currents(d=BASELINE, motors=None, servos=False):
    """One arm's 24 V branch (A): the four belt / inline joints from params (or `motors` {joint: (actuator, ratio)}), the
    DM-J4310 wrist pitch and, with `servos`, the two STS3215 behind the elbow node's 12 V buck (D14; on the M1 bench the
    buck hangs on the arm branch). Rows: (joint, actuator, rated-run A, peak-stall A).
    - sustained: every big joint at 50 % of (rated torque at rated speed) + the wrist at its nominal current (+ servos at 30 % of stall);
    - peak, drive limit = peak torque: the two worst joints stalled at peak + the other two at rated run + wrist peak (+ both servos stalled);
    - peak, drive limit = rated (bring-up's default, bringup.yaml `limit: rated`): every big joint at rated torque and rated
      speed at once + the wrist at rated (+ both servos stalled)."""
    js = dict(d.arm.joints)
    js.update(motors or {})
    rows = []
    for j in ("shoulder_pitch", "shoulder_roll", "upper_yaw", "elbow"):
        name, _ = js[j]
        g = act(name)
        rows.append((j, name, gim_bus_current(g["rated"], g["speed"], name), gim_bus_current(g["peak"], 0.0, name)))
    run = [r[2] for r in rows]
    stall_gain = sorted((r[3] - r[2] for r in rows), reverse=True)
    sv_pk = 2 * STS_STALL_A * 12 / V_MIN / BUCK_EFF if servos else 0.0
    w_pk, w_nom = J4310["i_peak"] * 24 / V_MIN * 0.5, J4310["i_nom"] * 24 / V_MIN * 0.5
    cont = 0.5 * sum(run) + J4310["i_nom"] * 0.5 + 0.3 * sv_pk
    peak = sum(run) + sum(max(x, 0.0) for x in stall_gain[:2]) + w_pk + sv_pk
    peak_rated = sum(run) + 2 * w_nom + sv_pk
    return dict(rows=rows, cont=cont, peak=peak, peak_rated=peak_rated, servos=sv_pk)


def branches(d=BASELINE):
    g = act("GIM6010-8")
    rated_run = gim_bus_current(g["rated"], g["speed"])        # rated torque at rated speed
    peak_stall = gim_bus_current(g["peak"], 0.0)                # peak torque, stalled (worst copper loss)
    ac = arm_currents(d)                                        # the arm's real motor set (inline GIM8108-36 roll by default)
    arm_cont, arm_peak = ac["cont"], ac["peak"]
    names = [r[1] for r in ac["rows"]]
    arm_name = " + ".join(f"{names.count(n)} × {n}" for n in dict.fromkeys(names)) + " + DM-J4310"
    base_cont, base_peak = 4 * gim_bus_current(0.8, 12.6), 2 * peak_stall + 2 * rated_run     # carpet cruise; threshold climb
    lift_cont, lift_peak = gim_bus_current(2.4, 12.6), peak_stall
    servo12 = 6 * 2.7 * 12 / V_MIN * 0.3                        # 6 × STS3215, 30 % of stall on average
    logic = (5 * 5 + 15) / V_MIN                                 # Pi 5 (5 V 5 A max) + cameras / lidar / CAN adapters
    # arm fuse 15 → 20 A slow-blow (Fable M2): the sustained figure is ≈ 12.5 A with the inline roll; 15 A left no margin
    rows = [("Base: 4 wheel cartridges", base_cont, base_peak, 20), (f"Left arm: {arm_name}", arm_cont, arm_peak, 20),
            (f"Right arm: {arm_name}", arm_cont, arm_peak, 20), ("Lift cartridge + pawl solenoid", lift_cont, lift_peak, 10),
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
          "GIM8108-8": 6.67,         # rpm/V at the output (openelab listing; aifitlab gives 6.04: the faster one is the worst case)
          "GIM8108-36": 62 / 24}     # ESTIMATE: no-load ≈ 1.3 × the 48 rpm rated (SDC303, 24 V); no published no-load speed
J_ROTOR = {"GIM6010-8": 6e-5, "GIM8108-8": 1.5e-4, "GIM8108-36": 1.5e-4}   # kg·m² at the motor shaft: ESTIMATE (outer rotor, Ø60/Ø81 class)
MOTOR_CU = {"GIM6010-8": (5.0 / 10.5, 0.35),        # output Kt (N·m/A), phase R (Ω): ESTIMATE (as GIM above)
            "GIM8108-8": (7.5 / 7.0, 0.25),         # 7.5 N·m rated at 7 A nominal (listing); R ESTIMATE
            "GIM8108-36": (3.86, 0.22)}             # SDC303: 3.86 N·m/A at the output, 9.2 A nominal, 34.1 A stall, 0.22 Ω (reseller spec tables, 2026-11-27 search)
COVER_KG = 0.5                       # D23 joint covers per arm (ESTIMATE, D23)
INERTIA_MARGIN = 1.15                # distributed-body inertia on top of the point masses (ESTIMATE)
# Clamp resistor. Was 3 × Vishay Dale RH-50 (aluminium-housed WIREWOUND: inductive, a few µH; Neon review #2 finding 1).
# Now 3 × Vishay Dale NH-50, the same housing with an Ayrton-Perry non-inductive winding; Vishay: the NH versions "have the
# same physical and electrical specifications" as the RH (50 W on the reference heatsink, short-time overload 5 × rated
# for 5 s; RH/NH datasheet 30201, Mouser RH/NH product page; resistance range down to 0.05 Ω, so 0.56 Ω exists).
# Price ESTIMATE: no 0.56 Ω NH-50 listing seen; IBS NH05050R00FE02 $15.01 (10-wk lead), Future NH05025R00FE02 $42.30
# (2026-11-27 search snapshots) → $30.
NH50 = dict(p_rated=50.0, overload=5.0, t_overload=5.0, price=30.0, L=1.0e-6)   # L: non-inductive winding + leads, ESTIMATE
RH50 = dict(p_rated=50.0, overload=5.0, t_overload=5.0, price=15.0, L=5.0e-6)   # RH: still the pre-charge R3 + dummy load (relay-switched); L ESTIMATE (2–5 µH, not published)
# Freewheel path (finding 1): a Schottky across each clamp resistor carries the winding + lead current when the MOSFET
# turns off, so the drain sees V_bus + V_F + L_loop·di/dt instead of L_res·di/dt.
FREEWHEEL = dict(name="Vishay VS-40CPQ060-N3 (60 V, 2 × 20 A common-cathode Schottky, TO-247AC, both legs in parallel)",
                 ifsm=3200.0, vf=0.6, price=1.9,
                 src="Vishay datasheet (IFSM 3200 A at 5 µs sine, VF 0.49 V at 20 A per leg); RS UK £37.00 per tube of 25 (2026-11-27 search)")
# Comparator (finding 1): the TLV3702 is a nanopower part, 36 µs propagation delay (TI product page); the TLV3202 is 40 ns
# typical, 55 ns max over −40…125 °C at 5 V / 15 pF (TI SBOS561C), push-pull, 2.7–5.5 V. Same dual pinout class.
COMP = dict(name="TI TLV3202 dual push-pull comparator (40 ns typ, 55 ns max)", t_pd=55e-9, src="TI SBOS561C")
COMP_OLD = dict(name="TI TLV3702 (nanopower)", t_pd=36e-6, src="TI TLV3702 product page: 36 µs")
CMP_FILTER, T_ON_MIN = 2e-6, 50e-6     # comparator divider RC; minimum clamp on-time (one-shot ORed into the gate): safety_board_sim.py
T_SW, L_LOOP = 0.3e-6, 30e-9         # gate-resistor-set MOSFET switching time; MOSFET–diode–film-cap loop inductance (ESTIMATE: tight layout)
IRFB7530_VDS = 60.0
# Thresholds (finding 1 asked for ~0.8 V of hysteresis): with ± 1 % on the threshold, 0.8 V cannot satisfy both "off above a full
# pack" (v_on·0.99 − h > 25.2 V) and "peak ≤ 28 − 0.5 V" (v_on·1.01 + h ≤ 27.5 V): those need h ≤ 0.6 V. Netlist v3 used 26.4 V / 0.6 V
# (on at 27.0, off at 25.8 V). Astra review #3 / rnd/safety_board_sim.py: (1) as drawn the clamp rings at MHz, because its own turn-on
# step at the board node (≈ 146 A × C1's ESR + the film cap's sag ≈ 2 V) is bigger than the 1.2 V window: a 2 µs divider filter and a
# 50 µs minimum on-time fix it, and the on-time (not the hysteresis) now sets the swing; (2) the drives sit 1–3 V above the board
# (regen current × harness R), so the window moves down: on at 26.0 V, off at 25.7 V (v_on 25.85 ± 0.15; ± 1 %: off ≥ 25.44 V > 25.2 V).
CLAMP = dict(v_on=25.85, v_tol=0.01, hyst=0.15, n_res=3, r_each=0.56, adiabatic=0.25,
             mosfet="3 × Infineon IRFB7530PbF (60 V, 2.0 mΩ max, 195 A package limit; one per resistor)")
N_DRIVERS = {"robot": 15, "M1": 5}   # 13 GIM + 2 DM-J4310; M1: 4 GIM + 1 DM-J4310
C_BULK, C_DRIVER = 4700e-6, 470e-6   # the bus capacitor + per-driver input capacitance (ESTIMATE, not published)
I_IDLE_DRIVER = 0.04                 # A per driver at 24 V, logic only (ESTIMATE)
# Contactor: TE Kilovac ALEV100 (24 V coil, 100 A, 1 Form A aux contact 2 A @ 30 VDC). LEV100 family: release 10 ms,
# operate 25 ms max (DigiKey / Xonelec listings of LEV100A5ANG; ALEV100 datasheet 2025-02). LEV100A5ANG ≈ $92 (Master Electronics).
CONTACTOR = dict(name="TE Kilovac ALEV100 (24 V coil, 1 Form A aux)", t_release=0.010, t_operate=0.025, price=110.0)
# Watchdog: SN74LVC1G123 retriggerable monostable on its own 3.3 V LDO, t_w ≈ R·C (TI SLVA720); retriggered by the
# heartbeat edge the Pi emits only after a complete healthy 500 Hz cycle (sw/safety.py heartbeat_ok()).
ROOT = Path(__file__).resolve().parent.parent
_BU = __import__("yaml").safe_load((ROOT / "sw" / "config" / "bringup.yaml").read_text())
JITTER = ROOT / "docs" / "sw" / "loop_jitter.json"     # python -m sw.loop_jitter (measured on this machine)
# Watchdog timeout: sw/config/bringup.yaml watchdog_s (one value for the board, the virtual bench and the bring-up check).
# Was 8 ms (four missed cycles): the measured loop (sw/loop_jitter.py) already reaches 7.9 ms between heartbeats with the
# cores loaded, so 8 ms would trip on jitter (Fable review M2) and every trip is a category-0 drop.
WATCHDOG = dict(t_nom=float(_BU.get("watchdog_s", 0.025)), tol=0.20, loop_hz=float(_BU["control_hz"]))
WD_RULE = dict(k_tail=3.0, k_max=2.0, step=0.005)   # timeout ≥ max(3 × p99.99, 2 × longest gap) of the loaded run, in 5 ms steps
SW2_BUDGET = 0.050                                  # heartbeat lost → no drive torque (SW2)
R_PRE, T_PRE_TIMEOUT = 3.3, 1.0      # switched pre-charge: RH-50 3.3 Ω through a 10 A / 30 VDC relay; 1 s timeout
T_FORCE = 0.100                      # forced bus discharge: a one-shot this long on the contactor's opening edge (not a level)
R_BLEED = 1000.0                     # permanent bus bleeder, 1 kΩ 3 W: bus to < 10 V in ≈ 7 s after any stop (no one-shot needed)
# Bench supply (Fable M2: the 10 A CC supply was a third of the arm's own peak). Mean Well SE-1000-24: 24 V 41.7 A, output
# adjustable (set 24.0–25.2 V). Overload (Neon review #2 check, 2026-11-27): the SE-1000 spec gives 105–125 % of rated power
# with "shut down o/p voltage, re-power on to recover" (Mean Well SE-1000 catalogue excerpt; the SE-1000-12 test report
# shows the shutdown at 116 %): a LATCHED shutdown, not constant-current limiting as this file said before.
# Prices: Bravo Electro $174.80, Master Electronics $170.13 (2026-11-27 search snapshots).
BENCH_PSU = dict(name="Mean Well SE-1000-24 (24 V 41.7 A, 1 kW)", i_max=41.7, trip=(1.05, 1.25), price=174.80,
                 src="Bravo Electro $174.80; Master Electronics $170.13 (2026-11-27 search snapshots)")
CC_PSU = dict(name="adjustable 0–30 V / 10 A CC/CV bench supply", i_max=10.0, price=65.71, src="Newegg generic 30 V 10 A CC/CV $65.71 (2026-11-27)")
BENCH_FUSE = 20                      # A, ATO on the arm lead (14 AWG silicone)
# Main fuse F1 (finding 1: a 40 A ATO carries 41.7 A, 104 %, indefinitely: Littelfuse ATO 110 % = 100 h minimum, 135 % =
# 0.75–600 s, 200 % = 0.15–5 s, 350 % = 0.08–0.5 s). Sized for the ROBOT pack now: ≥ 1.25 × the pack's worst sustained,
# carries the 74 A peak, and backs up the BMS's short-circuit trip. On the bench no fuse can operate before the supply's own
# latched shutdown (≤ 52 A), so the supply is the bench over-current device and the wiring is sized for its trip current.
MAIN_FUSE = dict(name="60 A MIDI bolt-down slow-blow (Littelfuse 498 class) + holder", rating=60, price=13.0, src="ESTIMATE")
ATO_CURVE = {1.10: (360000, None), 1.35: (0.75, 600), 2.00: (0.15, 5), 3.50: (0.08, 0.5)}   # Littelfuse ATO 3–40 A (s)
# Pack + BMS (Neon review #2 finding 11). The Lumenier NAV 6S 10 Ah 21700 (6S2P, XT60) is an FPV pack with no BMS: 35 A
# continuous at an 80 °C cut-off (20 A without one), 90 A burst (RaceDayQuads / GetFPV listings, 2026-11-27 search). The
# old external BMS was a "40 A JK class" ESTIMATE (trips on the 74 A peak). Now JK-B2A8S20P-HC: 4–8S, 200 A continuous,
# 350 A peak, 2 A active balance, CAN + RS485, two NTC inputs (nkon.nl €83.95; 18650batterystore $145, sold out).
PACK = dict(name="Lumenier NAV 6S 10 Ah 21700 (6S2P of 5 Ah cells, ESTIMATE)", cont=35.0, cont_nocut=20.0, burst=90.0, cells_p=2,
            dcir_cell=0.015, ocv50=3.65, price=150.0, src="racedayquads.com / getfpv.com listings (35 A cont. with 80 °C cut-off, 90 A burst)")
BMS = dict(name="JK-B2A8S20P-HC smart BMS (4–8S, 200 A cont. / 350 A peak, 2 A active balance, CAN + RS485, 2 × NTC)",
           cont=200.0, peak=350.0, price=95.0, src="nkon.nl €83.95 (≈ $95 incl. shipping share, ESTIMATE); 18650batterystore $145 (sold out)")
R_PATH = 0.010                       # BMS MOSFETs + XT90 + 10 AWG pack leads (Ω, ESTIMATE)
# Software pack-current budget (the supervisor sums the drives' estimated bus currents and scales the torque limits):
SW_PACK = dict(i_peak=70.0, i_avg=28.0, t_avg=30.0, t_cell_max=60.0)   # A instantaneous; A over a 30 s rolling mean; °C cell (BMS NTC)
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
        n = SPECS[name].get("gear_ratio", 8) * belt           # planetary (8:1; 36:1 inline roll) × belt
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


def clamp_check(rb, m1=None):
    """Regen clamp: sink power, pulse energy, thresholds, and (Neon review #2 finding 1) the switching behaviour: comparator
    delay, hysteretic switching frequency, MOSFET turn-off overshoot with and without the freewheel diode."""
    c = CLAMP
    R = c["r_each"] / c["n_res"]
    v_lo = c["v_on"] * (1 - c["v_tol"])
    v_hi = c["v_on"] * (1 + c["v_tol"]) + c["hyst"]
    P_cap = v_lo ** 2 / R                                            # sink power at the lowest threshold
    e_rating = NH50["overload"] * NH50["p_rated"] * NH50["t_overload"]   # 1,250 J per resistor (5 × 50 W for 5 s)
    t_pulse = max(r[6] for r in rb["rows"])                          # longest joint stop at peak torque (s)
    e_sqrt = e_rating * math.sqrt(min(t_pulse, NH50["t_overload"]) / NH50["t_overload"])   # heat penetration ∝ √t below 5 s (ESTIMATE)
    e_each_allow = min(c["adiabatic"] * e_rating, e_sqrt)
    e_allow = e_each_allow * c["n_res"]
    e_each = rb["E"] / c["n_res"]
    I_each = v_hi / c["r_each"]
    sw = {}
    for which, P in (("robot", rb["P"]), ("M1", (m1 or rb)["P"])):
        C = c_bus(which)
        i_reg = P / c["v_on"]                                         # worst regen current into the bus
        rise = i_reg / C                                              # V/s with the clamp off
        fall = v_hi / R / C                                           # V/s with the clamp on and no regen (worst undershoot)
        f_max = min((v_hi / R) / (4 * C * 2 * c["hyst"]), 1 / (2 * T_ON_MIN))   # hysteretic chopper (worst at i_reg = I_clamp / 2), bounded by the on-time
        sw[which] = dict(C=C, i_reg=i_reg, f_max=f_max,
                         over_new=rise * COMP["t_pd"], over_old=rise * COMP_OLD["t_pd"],
                         under_new=fall * COMP["t_pd"], under_old=fall * COMP_OLD["t_pd"],
                         p_sw=c["v_on"] * I_each * T_SW * f_max)       # MOSFET switching loss at f_max (each)
    v_peak = v_hi + max(x["over_new"] for x in sw.values())
    v_ds_diode = v_peak + FREEWHEEL["vf"] + L_LOOP * I_each / T_SW
    v_ds_rh = v_peak + RH50["L"] * I_each / T_SW                     # no freewheel path: the winding's L·di/dt lands on the drain
    v_ds_nh = v_peak + NH50["L"] * I_each / T_SW
    tau_fw = NH50["L"] / c["r_each"]
    return dict(R=R, v_lo=v_lo, v_hi=v_hi, v_peak=v_peak, I_pk=v_hi / R, I_each=I_each, P_cap=P_cap, e_rating=e_rating,
                e_allow=e_allow, e_each=e_each, e_each_allow=e_each_allow, e_sqrt=e_sqrt, t_pulse=t_pulse, p_avg_each=e_each / t_pulse,
                ok_P=P_cap >= rb["P"], ok_E=rb["E"] <= e_allow, ok_V=v_peak + 0.5 <= V_DRIVE_MAX, ok_off=v_lo - c["hyst"] > V_FULL,
                v_off_min=v_lo - c["hyst"] - max(x["under_new"] for x in sw.values()),
                v_off_min_old=26.5 * 0.99 - 0.4 - max(x["under_old"] for x in sw.values()),
                cont=c["n_res"] * NH50["p_rated"], sw=sw, v_ds_diode=v_ds_diode, v_ds_rh=v_ds_rh, v_ds_nh=v_ds_nh,
                tau_fw=tau_fw, i_fw=I_each, ok_ds=v_ds_diode <= 0.8 * IRFB7530_VDS, ok_fw=I_each <= FREEWHEEL["ifsm"] / 10,
                e_avalanche=0.5 * RH50["L"] * I_each ** 2)


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
            ("aux contact opens → discharge one-shot fires, clamp gate on", 0.0002),
            (f"bus {V_FULL:.1f} → {V_UVLO:.0f} V (driver UVLO, ESTIMATE) through the clamp resistors ({R:.2f} Ω × {C * 1e3:.1f} mF)", t_dis),
            ("phase current decays once the drive stops switching (ESTIMATE)", 0.001)]
    total = sum(t for _, t in rows)
    return dict(rows=rows, total=total, t_dis_passive=t_dis_passive, C=C,
                t_button=CONTACTOR["t_release"] + 0.0002 + t_dis + 0.001, t_sw=2 * t_cycle)


def precharge(which="robot"):
    C = c_bus(which)
    tau = R_PRE * C
    R_cl = CLAMP["r_each"] / CLAMP["n_res"]
    tau_dis = R_cl * C
    return dict(C=C, tau=tau, t95=3 * tau, i_in=V_FULL / R_PRE, p_in=V_FULL ** 2 / R_PRE, E=0.5 * C * V_FULL ** 2,
                droop=N_DRIVERS[which] * I_IDLE_DRIVER * R_PRE, e_fault=V_FULL ** 2 / R_PRE * T_PRE_TIMEOUT,
                e_rating=RH50["overload"] * RH50["p_rated"] * RH50["t_overload"],   # R3 stays an RH-50 (relay-switched, no fast edge)
                R_clamp=R_cl, v_fight=V_FULL * R_cl / (R_cl + R_PRE), tau_dis=tau_dis, n_tau_force=T_FORCE / tau_dis,
                E_dis=0.5 * C * V_FULL ** 2, t_bleed=R_BLEED * C * math.log(V_FULL / 10.0), i_bleed=V_FULL / R_BLEED,
                droop_bleed=V_FULL / R_BLEED * R_PRE)


def jitter():
    """Measured heartbeat intervals of the bring-up loop (sw/loop_jitter.py), or None if it has not been run."""
    import json
    if not JITTER.exists():
        return None
    return json.loads(JITTER.read_text())


def watchdog_required(js):
    """Smallest timeout the rule allows for this measurement (the loaded pass if there is one)."""
    iv = (js.get("loaded") or js)["interval_ms"]
    base = max(iv["p50"], iv["p9999"], iv["max"])
    t = max(WD_RULE["k_tail"] * iv["p9999"], WD_RULE["k_max"] * iv["max"]) / 1000.0
    return math.ceil(t / WD_RULE["step"] - 1e-9) * WD_RULE["step"], iv, base


def watchdog_lines(st, st1):
    ok = lambda b: "✅" if b else "❌"
    js = jitter()
    if js is None:
        return ["**Watchdog sizing:** run `python -m sw.loop_jitter` (no measurement in docs/sw/loop_jitter.json yet).", ""]
    req, iv, _ = watchdog_required(js)
    quiet = js["interval_ms"]
    ld = js.get("loaded")
    t_wd = WATCHDOG["t_nom"]
    tmo = float(_BU["can_timeout_s"])
    t_max = (SW2_BUDGET - (st["total"] - t_wd * (1 + WATCHDOG["tol"]))) / (1 + WATCHDOG["tol"])   # largest timeout inside SW2
    t_max = math.floor(t_max * 1e3) / 1e3
    L = [f"**Watchdog sized on a measurement (Fable M2).** `python -m sw.loop_jitter` ran sw/bringup.py's own hardware loop (RealClock, supervisor, codec, "
         f"python-can, 5 joints enabled) in wall-clock time on this machine ({js['machine']['cpus']} shared cores, non-real-time Linux): "
         f"{js['cycles']} heartbeat intervals, p50 {quiet['p50']:.2f} ms, p99.99 {quiet['p9999']:.2f} ms, longest **{quiet['max']:.2f} ms**"
         + (f"; with {ld['busy_processes']} busy processes competing, p99.99 {ld['interval_ms']['p9999']:.2f} ms, longest **{ld['interval_ms']['max']:.2f} ms** "
            f"({ld['over_ms']['8']} gaps over 8 ms)" if ld else "") + ". "
         f"The old 8 ms timeout was at the loaded longest gap: it would have tripped (a category-0 drop) on ordinary jitter. "
         f"Rule: timeout ≥ max({WD_RULE['k_tail']:.0f} × p99.99, {WD_RULE['k_max']:.0f} × longest) of the loaded run, in {WD_RULE['step'] * 1e3:.0f} ms steps "
         f"= **{req * 1e3:.0f} ms**; the stop chain must still meet the {SW2_BUDGET * 1e3:.0f} ms SW2 budget.", "",
         "| Check | Value | Limit | |", "|---|---|---|---|",
         f"| Watchdog timeout (bringup.yaml `watchdog_s`) | {t_wd * 1e3:.0f} ms | ≥ {req * 1e3:.0f} ms (measured rule) | {ok(t_wd >= req - 1e-9)} |",
         f"| Margin over the longest measured gap (loaded) | {t_wd * (1 - WATCHDOG['tol']) / (iv['max'] / 1e3):.1f}× at −{WATCHDOG['tol']:.0%} tolerance | ≥ 2× | "
         f"{ok(t_wd * (1 - WATCHDOG['tol']) >= 2 * iv['max'] / 1e3)} |",
         f"| Heartbeat lost → no drive torque (robot / M1) | {st['total'] * 1e3:.1f} / {st1['total'] * 1e3:.1f} ms | ≤ {SW2_BUDGET * 1e3:.0f} ms | "
         f"{ok(max(st['total'], st1['total']) <= SW2_BUDGET)} |",
         f"| Drive CAN timeout (bringup.yaml `can_timeout_s`; also sees the USB-CAN latency) | {tmo * 1e3:.0f} ms | ≥ the watchdog | {ok(tmo >= t_wd)} |", "",
         "Not measured here: the Pi 5 (slower cores) and a real USB-CAN adapter. Re-run `python -m sw.loop_jitter` on the Pi before day 1; "
         "the bring-up report also logs the longest heartbeat gap of the real loop (`heartbeat_gap_max_ms`). If either passes half the timeout, raise "
         f"`watchdog_s` (up to {t_max * 1e3:.0f} ms the SW2 budget still holds on the robot bus) or run the loop SCHED_FIFO on an isolated core.", ""]
    return L


def bench_supply_lines(d=BASELINE):
    """Fable M2: the M1 bench supply against the arm's own currents (inline roll, servos on the elbow-node buck)."""
    ok = lambda b: "✅" if b else "❌"
    a = arm_currents(d, servos=True)
    a20 = arm_currents(d, D20, servos=True)
    g = act("GIM6010-8")
    s1_servo = 0.0                                                # Neon #2 finding 15: the STS3215s moved to Step 2 (no 12 V rail in Step 1)
    s1_rated = gim_bus_current(g["rated"], g["speed"]) + J4310["i_nom"] * 24 / V_MIN + s1_servo
    s1_peak = gim_bus_current(g["peak"], 0.0) + J4310["i_peak"] * 24 / V_MIN * 0.5 + s1_servo
    P, CC = BENCH_PSU, CC_PSU
    trip_lo, trip_hi = P["i_max"] * P["trip"][0], P["i_max"] * P["trip"][1]
    L = ["### Bench supply (Fable M2; overload behaviour corrected after Neon review #2)",
         f"The kit had an {CC['name']} and a 15 A fuse against the arm's own 11.4 A sustained / 33.3 A peak (then with four GIM6010s): in CC mode a peak "
         "folds the bus back on 7 mF in milliseconds (a brown-out), and the fuse could never blow. The arm's currents with the real motor set "
         "(inline GIM8108-36 roll; the two STS3215 on the elbow node's 12 V buck, which hangs on the arm lead on the bench):", "",
         "| Case (bus current at the 20 V minimum) | Default arm (3 × GIM6010-8 + GIM8108-36) | D20 (hold A) | Step 1 (one GIM6010-8 + DM-J4310; servos in Step 2) |",
         "|---|---|---|---|",
         f"| Sustained (big joints 50 % of rated run, wrist nominal, servos 30 %) | {a['cont']:.1f} A | {a20['cont']:.1f} A | – |",
         f"| Peak, drive limit = **rated** (bring-up default `limit: rated`): every joint at rated torque and speed at once, servos stalled | {a['peak_rated']:.1f} A | {a20['peak_rated']:.1f} A | {s1_rated:.1f} A |",
         f"| Peak, drive limit = peak: two joints stalled at peak + the rest at rated run + wrist peak + servos stalled | {a['peak']:.1f} A | {a20['peak']:.1f} A | {s1_peak:.1f} A |", "",
         f"**Supply: {P['name']}** (${P['price']:.2f}; {P['src']}), set to 24.0–25.2 V. **Overload is a latched shutdown, not current limiting:** the SE-1000 spec gives "
         f"{P['trip'][0]:.0%}–{P['trip'][1]:.0%} of rated power ({trip_lo:.1f}–{trip_hi:.1f} A) with \"shut down o/p voltage, re-power on to recover\" (the SE-1000-12 test report "
         "trips at 116 %). Earlier text said constant-current limiting; that was wrong. Consequences: a peak over the trip collapses the bus (the drives brown out: a "
         "category-0 drop the catch cradle covers), the safety-board logic loses power and **must power up disarmed** (power-on reset on the latch, netlist below), and "
         f"the bench wiring only has to carry {trip_hi:.0f} A for the moment the supply needs to trip. "
         f"The {CC['name']} stays for the 1 A current-limited first power-up only (drives disabled).", "",
         "| Check | Value | Limit | |", "|---|---|---|---|",
         f"| Sustained, default arm | {a['cont']:.1f} A | ≤ {P['i_max']:.1f} A / 2 (continuous derating) | {ok(a['cont'] <= P['i_max'] / 2)} |",
         f"| Peak with the drive limits at rated (how bring-up runs) | {a['peak_rated']:.1f} A | ≤ {trip_lo:.1f} A, the lowest trip ({trip_lo / a['peak_rated']:.2f}×) | {ok(a['peak_rated'] <= trip_lo)} |",
         f"| Peak with the drive limits at peak (strength tests) | {a['peak']:.1f} A | ≤ {trip_lo:.1f} A ({trip_lo / a['peak']:.2f}×) | "
         f"{'✅' if a['peak'] <= trip_lo * 0.9 else '⚠️ no margin: peak-limit tests one joint at a time'} |",
         f"| Step 1 peak (one GIM at peak, wrist peak) | {s1_peak:.1f} A | ≤ {trip_lo:.1f} A | {ok(s1_peak <= trip_lo)} |",
         f"| Arm-lead fuse F3 (ATO, 14 AWG silicone) | {BENCH_FUSE} A | ≥ 1.25 × sustained {a['cont']:.1f} A | {ok(1.25 * a['cont'] <= BENCH_FUSE)} |",
         f"| Can any fuse clear a bench fault before the supply trips? | no: F3 at {trip_hi:.0f} A is {trip_hi / BENCH_FUSE:.1f}× (ATO opens in 0.15–5 s at 2×); the supply trips in ms | "
         "the supply is the bench over-current device; fuses protect the robot | ✅ (by design) |",
         f"| Supply → board wiring (10 AWG silicone, 5.3 mm²) | carries {trip_hi:.0f} A until the trip | ≥ {trip_hi:.0f} A short-time | ✅ |", "",
         "Rule for the bench: the drive torque limits stay at **rated** (bring-up's default) except in a strength test, which runs **one joint at a time**. "
         f"The bulk capacitance ({c_bus('M1') * 1e3:.1f} mF with the drivers') only rides through edges: 10 A over the supply rating drains it "
         f"{10 / c_bus('M1') / 1000:.1f} V per ms, so it is not a substitute for supply current. Regen into the supply is blocked by the ideal-diode stage and dumped by the clamp. "
         "On the robot the pack, its BMS and the software pack-current budget set the limit (*Pack and BMS* above).", ""]
    return L


def safety_board_lines():
    """Fable M2 + Neon review #2 finding 1: the mandatory bench safety board as a netlist + block diagram + state sequence
    (e-stop, contactor, pre-charge, clamp, watchdog, ideal diode). Part numbers are suggestions; values follow from the checks above."""
    t_wd = WATCHDOG["t_nom"]
    r_wd = t_wd / 100e-9                 # SN74LVC1G123: t_w ≈ K·R·C, K ≈ 1 (TI SLVA720), C = 100 nF C0G
    r_os = T_FORCE / 1e-6                # discharge one-shot: C = 1 µF C0G/film
    art = [
        "```",
        "PSU+ ─F1 60A MIDI─┬─[U1 LM74700 + Q6 IRFB7530: ideal diode]─ VIN ──┬───────────── K1 main (ALEV100) ─────────────┬─ VBUS ─F3 20A─► arm lead",
        "                  │                                               └─ K2 contact ─ R3 3.3Ω RH-50 (pre-charge) ───┤",
        "                  │                                                                                             ├─ C1 4.7mF 35V ┬ C2 10µF film ┬ D3 SMDJ33A (edge snubber)",
        "                  │                                                                                             ├─ R6 1kΩ 3W bleeder (permanent)",
        "                  │                                                                                             ├─ R1–R3 0.56Ω NH-50 ∥ D4–D6 VS-40CPQ060 ── Q1–Q3 IRFB7530 (R_G) ── GND",
        "                  │                                                                                             └─ divider + 2µs RC ─► U4 TLV3202: OV (on 26.0 / off 25.7V) ∨ U16 50µs min on-time; PRE_OK (VBUS ≥ VIN−3V)",
        "                  └─F2 1A─ U2 24→12V buck ─ U3 3.3V LDO (watchdog / logic only)",
        "",
        "Coil loop (hardware only):  +24V_L ─ S1a NC ─ S2a NC ─ K1 coil (D1 + Z1 suppression) ─ Q4 drain;  Q4 gate = U8: LATCH ∧ WD_OK ∧ (PRE_OK ∨ AUX)",
        "K_PRE:                      +24V_L ─ S1a/S2a (same chain) ─ K2 coil ─ Q5;  Q5 gate = K2_DRIVE = LATCH ∧ ¬AUX ∧ ¬PRE_TIMEOUT",
        "Clamp gate:                 U7 OR( OV , FORCE ) → U9 UCC27524 → R_G → Q1–Q3;   FORCE = U6 one-shot, Ā = AUX (falling edge), B = 3V3, CLR̄ = ¬K2_DRIVE (U15)",
        "Watchdog:                   GPIO17 heartbeat ─ U5a 74LVC1G17 ─ U5 SN74LVC1G123 B (rising edge), Ā = GND, CLR̄ = 3V3 (R4·C3) → WD_OK",
        "Monitored reset:            SB1 ARM (NO) → U10 SN74LVC1G74 D-FF set only if S1b, S2b closed ∧ WD_OK; CLR̄ low on any STOP, FORCE (any K1 opening), TH1, PRE_TIMEOUT, power-on (R9·C9)",
        "Feedback to the Pi:         K1 aux (NO) ─ U12 PC817 ─ AUX (logic, high = closed) + GPIO27 (AUX_OK);  S1b/S2b ch2 ─ U13 PC817 ─ GPIO22 (ESTOP_SENSE)",
        "```"]
    nets = [
        ("F1", MAIN_FUSE["name"], "PSU+ / pack+ → F1_OUT", "sized for the robot pack (≥ 1.25 × 37 A sustained, carries the 74 A peak, backs up the BMS short-circuit trip); "
                                                         "on the bench the SE-1000-24's latched shutdown (43.8–52 A) acts first"),
        ("U1, Q6", "LM74700-Q1 ideal-diode controller + IRFB7530 (60 V, 2 mΩ)", "F1_OUT → VIN", "blocks regen into the PSU (a PSU cannot sink)"),
        ("K1", "TE Kilovac ALEV100, 24 V coil, 100 A, 1 Form A aux", "VIN ↔ VBUS; coil COIL+ / COIL−; aux AUX_A / AUX_B", "main contactor (category-0 stop)"),
        ("D1, Z1", "1N4007 + 24 V Zener (e.g. 1N5359B) in series across the K1 coil", "COIL+ / COIL−", "coil suppression that keeps the release time short (diode alone slows it)"),
        ("Q4", "**100 V** logic-level N-MOSFET (IRL540N class; was the 30 V IRLB8721: the D1 + Z1 suppression puts ≈ 52 V on the drain at every turn-off, "
               "safety_board_sim.md)", "COIL− → GND, gate U8_OUT", "coil driver: closes on LATCH ∧ WD_OK ∧ PRE_OK, then holds on AUX (finding 1: PRE_OK alone in the hold term)"),
        ("U8", "SN74LVC1G11 3-input AND + SN74LVC1G32 OR (PRE_OK ∨ AUX)", "LATCH, WD_OK, PRE_OK, AUX → Q4 gate", "close on a charged bus, hold on the aux contact"),
        ("S1, S2", "2 × two-channel latching mushroom, 2 NC each", "S1a/S2a: +24V_L → COIL+ (series); S1b/S2b: 3V3 → U10 / U13", "ch1 opens the coil loop directly; ch2 tells the logic and the Pi"),
        ("SB1", "ARM / reset pushbutton, NO", "3V3 → U10 CLK", "monitored manual reset (never re-closes on its own)"),
        ("U10", "SN74LVC1G74 D flip-flop", "D = S1b ∧ S2b ∧ WD_OK ∧ TH1 ∧ **¬AUX** (a stuck AUX would bypass pre-charge), CLK = SB1, CLR̄ = ¬(STOP ∨ FORCE ∨ POR ∨ DISC ∨ UV12 ∨ OC)", "LATCH: armed state; **any K1 opening clears it** (FORCE fires on every aux falling edge), so K2 cannot re-close by itself"),
        ("R9, C9", "100 kΩ + 1 µF (τ 0.1 s) on U10 CLR̄", "3V3 → CLR̄", "power-on reset: after a supply trip the board powers up disarmed"),
        ("K2, Q5", "10 A / 30 VDC relay (24 V coil, e.g. Omron G5LE-1) + IRLML6344", "VIN → K2_C → R3 → VBUS; gate = K2_DRIVE", "pre-charge, opens as soon as K1's aux closes"),
        ("R3", f"Vishay RH-50 {R_PRE} Ω (relay-switched: no fast edge, wirewound is fine)", "K2_C → VBUS", f"pre-charge resistor (τ {R_PRE * c_bus('M1') * 1e3:.0f} ms on the M1 bus)"),
        ("U11", "RC + comparator (or a third LVC1G123), 1 s", "starts on LATCH ↑, output PRE_TIMEOUT clears U10 if AUX is still open", "pre-charge timeout (latched fault)"),
        ("U4", f"{COMP['name']} + TL431B reference", f"VBUS divider (0.1 % resistors) + {CMP_FILTER * 1e6:.0f} µs RC → OV (on {CLAMP['v_on'] + CLAMP['hyst']:.1f} V, "
               f"off {CLAMP['v_on'] - CLAMP['hyst']:.1f} V by positive feedback; was 27.0 / 25.8 V) and PRE_OK (VBUS ≥ VIN − 3 V)",
         "clamp threshold; closes K1 only on a charged bus (was the 36 µs TLV3702)"),
        ("U16", f"SN74LVC1G123 one-shot, {T_ON_MIN * 1e6:.0f} µs", "B = OV (rising), output ORed into U7", "minimum clamp on-time: as drawn the 55 ns comparator rang at MHz on the "
               "clamp's own ESR step (safety_board_sim.md)"),
        ("U17", "K1 discrepancy monitor: SN74LVC1G86 XOR + RC (30 ms) + comparator", "XOR(Q4 gate, AUX) → DISC", "coil command ≠ AUX for > 30 ms (welded K1, broken / stuck AUX, "
               "K1 failed to close) → FORCE (U6 trigger) and LATCH cleared; with a welded K1 the clamp loads the bench supply until it latches off"),
        ("U18", "12 V UVLO (TLV3012-class comparator + reference, 10.5 V)", "U2 output → UV12", "no gate-driver supply → STOP (a dead U2 otherwise leaves K1 closed with no clamp)"),
        ("U19", "ACS724-30A Hall sensor + 5 s RC + comparator (16 A)", "VBUS → F3 branch → OC", "arm-branch overload trip: the 20 A slow-blow leaves the in-arm 16 AWG "
               "unprotected at 18–35 A (Step 2 gate; safety_board_sim.md *Fuse coordination*)"),
        ("U6", f"SN74LVC1G123 one-shot, R {r_os / 1e3:.0f} kΩ, C 1 µF", "**Ā = AUX** (falling edge = K1 opened), B = 3V3, CLR̄ = ¬K2_DRIVE",
         f"FORCE: {T_FORCE * 1e3:.0f} ms forced discharge on the opening edge only (B is the rising-edge input: finding 1), never during pre-charge"),
        ("U15", "SN74LVC1G04 inverter", "K2_DRIVE → U6 CLR̄", "the old netlist tied CLR̄ to K2_DRIVE itself, which held the one-shot cleared whenever K2 was off: FORCE could never fire"),
        ("U7, U9", "SN74LVC1G32 OR + UCC27524 gate driver (12 V)", "OV ∨ FORCE → R_G → Q1–Q3 gates", "clamp gate"),
        ("R_G", f"gate resistors (≈ 10–22 Ω, set on the scope for ≈ {T_SW * 1e6:.1f} µs edges)", "U9 → Q1–Q3", "limits di/dt: drain overshoot ≈ 5 V with the 30 nH loop"),
        ("Q1–Q3 / R1–R3", "3 × IRFB7530 / 3 × **Vishay NH-50 0.56 Ω** on an Al plate, 10 AWG leads", "VBUS → R → Q → GND", "regen clamp + bus discharge (0.19 Ω; robot peak regen 2.8 kW)"),
        ("D4–D6", FREEWHEEL["name"], "anode at Q drain, cathode at VBUS (across each resistor)", "freewheel path for the resistor + lead inductance at every turn-off (finding 1)"),
        ("TH1", "85 °C NC thermal switch on the resistor plate", "in series with U10 D (opens → STOP)", "clamp over-temperature latches a stop"),
        ("R6", f"{R_BLEED / 1000:.0f} kΩ 3 W", "VBUS → GND", "permanent bleeder (bus < 10 V within ≈ 7–11 s after any stop, M1 / robot)"),
        ("C1, C2, D3", "4.7 mF 35 V electrolytic + 10 µF film at Q1–Q3 + SMDJ33A TVS", "VBUS → GND", "bulk, switching overshoot, edge snubber only (not the clamp)"),
        ("U5", f"SN74LVC1G123, R4 {r_wd / 1e3:.0f} kΩ 1 %, C3 100 nF C0G", "B = heartbeat (rising edges; GPIO17 via 74LVC1G17), Ā = GND, Q = WD_OK",
         f"hardware watchdog, {t_wd * 1e3:.0f} ms (bringup.yaml watchdog_s): needs ≥ 1 rising edge per timeout (a 500 Hz toggle gives one every 4 ms)"),
        ("U2, U3", "24 → 12 V 1 A buck + 3.3 V LDO (e.g. MCP1703A-3302)", "F2 (1 A) from VIN", "logic supply of its own (the watchdog must not share the Pi's rail); nothing else may hang on it"),
        ("U12, U13", "2 × PC817 opto", "K1 aux → AUX (to U6 / U8 / U11) and GPIO27 (AUX_OK); S1b/S2b → GPIO22 (ESTOP_SENSE)", "sw/bringup.py `--estop gpiod:gpiochip0:17:27`; 22 for the button sense"),
        ("F3", f"{BENCH_FUSE} A ATO", "VBUS → arm lead (14 AWG)", "arm branch (sustained ≈ 15 A, Bench supply above)"),
    ]
    seq = [
        ("OFF / STOPPED", "K1 open, K2 open, LATCH 0 (also after power-up: R9·C9)", "after any stop: FORCE fires once for 100 ms, then the bleeder holds VBUS near 0 V"),
        ("ARM pressed", "needs both mushrooms released (S1b, S2b), WD_OK (the Pi already toggles the heartbeat), TH1 closed", "LATCH ← 1; K2 closes; U11 timer starts"),
        ("PRE-CHARGE", f"VBUS rises through {R_PRE} Ω, τ ≈ {R_PRE * c_bus('M1') * 1e3:.0f} ms (M1)", f"FORCE inhibited (no aux edge, and CLR̄ held low while K2 is driven); OV never trips below {CLAMP['v_on'] * (1 - CLAMP['v_tol']) + CLAMP['hyst']:.2f} V"),
        ("RUN", "PRE_OK → Q4 on → K1 closes; aux closes → K2 drops out; Q4 now holds on AUX", "drives powered; a bus sag cannot open K1 (PRE_OK has left the hold term), the bring-up enables the drives only after this"),
        ("STOP (any)", "mushroom, WD_OK low, TH1, PRE_TIMEOUT, the Pi (stops the heartbeat), or K1 dropping out for any other reason (coil dip, shock)",
         "Q4 off → K1 opens (≤ 10 ms) → aux falls → FORCE 100 ms (bus < UVLO in ≈ 2 ms) **and LATCH cleared** → K2 stays open; only SB1 re-arms"),
    ]
    L = ["### Safety board (Fable M2; Neon review #2 finding 1 fixes)",
         "Block diagram and netlist of the mandatory bench board. It is a design to be built and reviewed on perfboard / a 2-layer board, not a certified "
         "safety function; part numbers are suggestions with the ratings the checks above need. **Draw it in KiCad and simulate the clamp loop before ordering the board parts** "
         "(M1_READINESS Step 1 gate).", "",
         "**Neon review #2, finding 1, checked against this netlist:** (a) RH-50 is wirewound and there was no freewheel path: **confirmed** (NH-50 + Schottky, table above); "
         "(b) TLV3702 delay: **confirmed** (36 µs; now TLV3202, 55 ns max); the \"10 kHz banging\" is ordinary hysteretic chopping once the diodes are in; "
         "(c) chatter from PRE_OK on a bus sag: **not as described** (PRE_OK compares VBUS with VIN, and with K1 closed they are the same node, so a sag moves both), but the "
         "logic had a real hole: if K1 dropped out for any reason with LATCH still high, K2 re-closed and the board **re-armed itself** without the reset button: now K1 holds on "
         "AUX and any opening clears LATCH; (d) U6 on pin B: **confirmed** (B is rising-edge, Ā falling-edge, TI SCES586; now Ā), and U6's CLR̄ polarity was also inverted "
         "(U15); (e) the 40 A fuse: **confirmed** that it cannot clear 41.7 A, and the SE-1000-24 does not current-limit at all: it shuts down at 105–125 % (*Bench supply* above).", ""] + art + [
         "", "| Ref | Part | Nets | Function |", "|---|---|---|---|"] + [f"| {a} | {b} | {c} | {d} |" for a, b, c, d in nets] + [
         "", "| State | Condition | Effect |", "|---|---|---|"] + [f"| {a} | {b} | {c} |" for a, b, c in seq] + [
         "", "Bench tests of the board before any motor (bring-up step 0): ARM refused with a mushroom pressed or without a heartbeat; pre-charge "
         "reaches VIN − 3 V and K1 closes; a 10 Ω RH-50 dummy load across VBUS holds it under VIN − 3 V and trips PRE_TIMEOUT; heartbeat stopped → K1 opens within "
         f"{t_wd * 1.2 * 1e3:.0f} ms; FORCE pulse seen on the scope at every opening and never during pre-charge; **K1 coil interrupted by hand while armed → LATCH clears and "
         "K2 stays open** (no self re-arm); supply switched off and on → board comes up disarmed; clamp at 26.0 V from a lab supply through 1 Ω with the drain voltage "
         "on the scope (≤ 48 V). **The full procedure with pass/fail numbers is [safety_board_tests.md](safety_board_tests.md)** (Step-1 gate).", ""] + sim_lines()
    return L


def sim_lines():
    """Summary of rnd/safety_board_sim.py (ngspice + logic model), read from its cached JSON (Astra review #3, critical 3)."""
    p = DOCS / "safety_board_sim.json"
    if not p.exists():
        return ["*Run `python -m rnd.safety_board_sim` for the board simulation.*", ""]
    import json
    R = json.loads(p.read_text())
    dm = {(d["which"], d["variant"], d["scale"], d["envelope"], d["corner"]): d for d in R["dumps"]}
    g = lambda w, v="fix", sc=1.0, e=True, c="high": dm.get((w, v, sc, e, c), {})
    coil = R["coil"]
    dis = {d["which"]: d for d in R["discharge"]}
    sc = R["stop_chain"]
    pre = [x for x in R["precharge"] if not x.get("timeout")]
    logic_bad = [x for x in R["logic"] if x["variant"] == "v3 as drawn" and not x["ok"]]
    fuse_bad = [f for f in R["fuses"] if not f["protected"]]
    L = ["### Safety board simulation (Astra review #3, critical 3)",
         f"`python -m rnd.safety_board_sim` ({R['method'].split(' /')[0]}; [safety_board_sim.md](safety_board_sim.md)): the clamp with the harness to every drive, "
         "pre-charge, the coil drop and the forced discharge as circuits; the logic as an event model of this netlist; fuses against the wires' heating. "
         "A model of an unbuilt board: it changed the netlist above and sets the bench numbers, it does not show that the built board works.", "",
         "| Check | Netlist v3 as drawn | With the changes | |", "|---|---|---|---|",
         f"| Clamp chopper (M1, 1.3 kW regen) | rings at {g('M1', 'v3').get('f_sw_mean', 0) / 1e6:.0f} MHz (simulation aborts): turn-on step ≈ 2 V > 1.2 V window | "
         f"{g('M1').get('f_sw_mean', 0) / 1e3:.1f} kHz with the {CMP_FILTER * 1e6:.0f} µs filter + {T_ON_MIN * 1e6:.0f} µs on-time | ✅ |",
         f"| Highest drive voltage, rated-limit stop (×0.45) M1 / robot | 27.0 V window (v3, with the filter): {g('M1', 'v3+ton').get('v_drive_max', float('nan')):.2f} V already at full regen | "
         f"{g('M1', sc=0.45).get('v_drive_max', float('nan')):.2f} / {g('robot', sc=0.45).get('v_drive_max', float('nan')):.2f} V (≤ 27.5) | ✅ |",
         f"| Highest drive voltage, every joint at peak torque at once M1 / robot | – | {g('M1').get('v_drive_max', float('nan')):.2f} / {g('robot').get('v_drive_max', float('nan')):.2f} V "
         f"(elbow, behind ≈ 0.1 Ω of harness) | ❌ → regen ≤ {R['regen_limit']['M1']['P_max']:.0f} W (M1) / {R['regen_limit']['robot']['P_max']:.0f} W (robot) |",
         f"| Resistor energy, full worst-case dump | – | {g('M1').get('e_res_each', 0):.0f} J (M1) / {g('robot').get('e_res_each', 0):.0f} J (robot, 323 J) per NH-50 ≤ 312 J | ✅ |",
         f"| MOSFET drain peak | – | {max(g('M1').get('v_drain_max', 0), g('robot').get('v_drain_max', 0)):.1f} V ≤ 48 V | ✅ |",
         f"| K1 coil driver drain at turn-off (D1 + Z1) | {coil[0]['v_drain_max']:.0f} V on a 30 V part (avalanche every stop) | {coil[1]['v_drain_max']:.0f} V on a 100 V part | ✅ |",
         f"| Pre-charge: PRE_OK / K1 making current M1, robot | " + " / ".join(f"{x['t_pre_ok'] * 1e3:.0f} ms, {x['i_make']:.0f} A" for x in pre[:2]) +
         " | robot idle draw must stay < 0.6 A (×1.5 → PRE_TIMEOUT) | ⚠️ |",
         f"| K1 open → far drive < UVLO M1 / robot | {dis['M1']['t_uvlo_far'] * 1e3:.1f} / {dis['robot']['t_uvlo_far'] * 1e3:.1f} ms | – | ✅ |",
         f"| Heartbeat lost → no torque, worst tolerances (M1) | {sc['M1']['total_datasheet'] * 1e3:.1f} ms with the datasheet release ({sc['M1']['total'] * 1e3:.1f} coil model) | ≤ 50 | ✅ |",
         f"| Logic single faults | {len(logic_bad)} fail: " + "; ".join(x["case"] for x in logic_bad) + " | all pass (¬AUX on ARM, discrepancy monitor, 12 V UVLO) | ✅ |",
         f"| Wire protection | " + "; ".join(f"{f['case'].split(':')[0]} unprotected at {min(f['gap']):.0f}–{max(f['gap']):.0f} A" for f in fuse_bad[:1]) +
         " | branch current monitor (U19) | ✅ |", "",
         "**Regen requirement (sw / planner):** the board clamps its own node; each drive sits above it by its regen current × harness resistance. A controlled "
         f"stop may return at most ≈ {R['regen_limit']['M1']['P_max']:.0f} W on the M1 bench arm ({R['regen_limit']['robot']['P_max']:.0f} W on the robot, both arms + lift): "
         "drive limits at rated (bring-up's default) meet it; a peak-limit strength test runs one joint at a time (already the bench rule); the planner's "
         "deceleration limit must keep the sum of τ·ω of the braking joints under it. Board test T12 scopes the bus **at the drive**.", ""]
    return L


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
    m1 = regen_budget(d, payload=2.0, arms=1, lift=False)
    cc = clamp_check(rb, m1)
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
          f"The clamp's continuous rating is {cc['cont']:.0f} W ({CLAMP["n_res"]} NH-50 on their reference heatsink), so **lowering is capped at the rated speed whenever the pack is full or the "
          "supply is a bench PSU** (planner rule; the clamp's thermal switch latches a fault if it is ignored).", "",
          f"**Part (Neon review #2 finding 1):** {CLAMP['n_res']} × **Vishay Dale NH-50, {CLAMP['r_each']} Ω** (the RH-50's housing with a non-inductive Ayrton-Perry winding: "
          f"Vishay gives the NH the same ratings as the RH, 50 W on its heatsink, short-time overload {NH50['overload']:.0f} × rated for {NH50['t_overload']:.0f} s = "
          f"**{cc['e_rating']:.0f} J per resistor**) in parallel = {cc['R']:.2f} Ω, switched by {CLAMP['mosfet']}, **each resistor with a {FREEWHEEL['name']} across it** "
          f"(freewheel path), a {COMP['name']} with a TL431B reference (on {CLAMP['v_on'] + CLAMP['hyst']:.1f} V, off {CLAMP['v_on'] - CLAMP['hyst']:.1f} V, ± 1 %; {CMP_FILTER * 1e6:.0f} µs filter, {T_ON_MIN * 1e6:.0f} µs minimum on-time), a UCC27524 gate driver with gate resistors "
          f"that set ≈ {T_SW * 1e6:.1f} µs switching edges, 10 µF film capacitors at the MOSFETs, and an 85 °C NC thermal switch on the resistor plate. "
          "**Mandatory on the M1 bench** (a bench PSU cannot sink current). The 33 V TVS stays only as an edge snubber: a TVS that does not conduct at 25.2 V "
          "clamps far above 28 V at current, so it cannot enforce the drive limit (Astra 17). On the bench, an ideal-diode stage stops regen reaching the PSU.", "",
          "| Check | Value | Limit | |", "|---|---|---|---|",
          f"| Sink power at the lowest threshold ({cc['v_lo']:.2f} V² / {cc['R']:.2f} Ω) | {cc['P_cap']:.0f} W | ≥ worst-case peak {rb['P']:.0f} W | {ok(cc['ok_P'])} |",
          f"| Pulse energy per resistor ({rb['E']:.0f} J / {CLAMP['n_res']}) over the longest stop ({cc['t_pulse']:.2f} s; mean {cc['p_avg_each']:.0f} W = "
          f"{cc['p_avg_each'] / NH50['p_rated']:.1f} × rated) | {cc['e_each']:.0f} J | ≤ min(25 % of the 5 s rating, rating × √(t / 5 s)) = {cc['e_each_allow']:.0f} J (ESTIMATE; "
          f"{cc['e_each_allow'] / cc['e_each']:.1f}×) | {ok(cc['ok_E'])} |",
          f"| Highest bus voltage while clamping (+1 % tol. + hysteresis + comparator delay × dV/dt) | {cc['v_peak']:.2f} V | ≤ {V_DRIVE_MAX:.0f} V − 0.5 V | {ok(cc['ok_V'])} |",
          f"| Clamp off at a full pack (lowest threshold − hysteresis) | {cc['v_lo'] - CLAMP['hyst']:.2f} V (with the comparator delay {cc['v_off_min']:.2f} V) | > {V_FULL} V | {ok(cc['ok_off'] and cc['v_off_min'] > V_FULL)} |",
          f"| Peak current (MOSFETs, wiring) | {cc['I_pk']:.0f} A ({cc['I_each']:.0f} A per MOSFET) | 195 A package limit each | ✅ |",
          f"| MOSFET drain at turn-off, **RH-50 without a freewheel diode** (L ≈ {RH50['L'] * 1e6:.0f} µH × {cc['I_each']:.0f} A / {T_SW * 1e6:.1f} µs) | {cc['v_ds_rh']:.0f} V "
          f"(avalanche, {cc['e_avalanche'] * 1e3:.1f} mJ per edge, thousands of edges per second; an NH-50 alone, ≈ {NH50['L'] * 1e6:.0f} µH with leads, still {cc['v_ds_nh']:.0f} V) | ≤ {0.8 * IRFB7530_VDS:.0f} V (80 % of 60 V) | ❌ (the old design) |",
          f"| MOSFET drain at turn-off, NH-50 + freewheel Schottky (V_bus + V_F + {L_LOOP * 1e9:.0f} nH loop × di/dt) | {cc['v_ds_diode']:.1f} V | ≤ {0.8 * IRFB7530_VDS:.0f} V | {ok(cc['ok_ds'])} |",
          f"| Freewheel pulse per diode | {cc['i_fw']:.0f} A decaying with L/R ≈ {cc['tau_fw'] * 1e6:.1f} µs (NH-50 L ESTIMATE) | IFSM {FREEWHEEL['ifsm']:.0f} A (5 µs); ≥ 10× | {ok(cc['ok_fw'])} |", "",
          "**Comparator speed and switching (finding 1, re-derived):** the clamp is a hysteretic brake chopper: it switches on at the upper threshold, "
          "discharges the bus by the hysteresis, switches off, and repeats while regen lasts; switching at kHz rates is normal for a chopper, the failure was the "
          "missing freewheel path. Worst-case frequency is at a regen current of half the clamp current: f = I_clamp / (4 · C · ΔV).", "",
          "| Bus | C | Worst regen current | Overshoot from the comparator delay (TLV3702 → TLV3202) | Undershoot after switch-off (TLV3702 → TLV3202) | f_sw max | MOSFET switching loss at f_max (each) |",
          "|---|---|---|---|---|---|---|"] + [
          f"| {w} | {x['C'] * 1e3:.1f} mF | {x['i_reg']:.0f} A | {x['over_old'] * 1e3:.0f} → {x['over_new'] * 1e3:.1f} mV | {x['under_old'] * 1e3:.0f} → {x['under_new'] * 1e3:.1f} mV | "
          f"{x['f_max'] / 1e3:.1f} kHz | {x['p_sw']:.1f} W |" for w, x in cc["sw"].items()] + ["",
          f"With the TLV3702's 36 µs the old thresholds (26.5 V, 0.4 V) would have dipped to {cc['v_off_min_old']:.2f} V, under a full pack (the clamp then drains the supply / pack), "
          f"and overshot by up to {max(x['over_old'] for x in cc['sw'].values()):.2f} V; with the TLV3202 both are negligible. 0.8 V of hysteresis (the review's suggestion) "
          "does not fit inside 25.2 … 27.5 V with a ± 1 % threshold. **Astra review #3 (rnd/safety_board_sim.py, ngspice):** the fast comparator alone rings at MHz, "
          "because the clamp's own turn-on step at the board node (≈ 146 A × C1's ESR plus the film capacitor's sag, ≈ 2 V) exceeds the window; a "
          f"{CMP_FILTER * 1e6:.0f} µs divider filter and a {T_ON_MIN * 1e6:.0f} µs minimum on-time (U16) bound the chopper to a few kHz, and the window moved "
          f"down to on {CLAMP['v_on'] + CLAMP['hyst']:.1f} / off {CLAMP['v_on'] - CLAMP['hyst']:.1f} V because the drives sit above the board by their regen current × harness "
          "resistance (*Safety board simulation* below).", "",
          "Check on the restrained rig (Astra 17): a dummy capacitive energy source first, then one axis; scope the bus **at the drive** and the **MOSFET drains** for a simultaneous stop, "
          "a PSU trip and a contactor opening. Confirm the NH-50 pulse capability for a ≈ 5× / 0.4 s pulse with Vishay (the 5 s rating and the √t scaling are not a pulse curve), "
          "and simulate the clamp loop (LTspice: bus C, NH-50 + lead L, diode, MOSFET, gate R) before ordering the board.", "",
          "### Pre-charge, switched (not in parallel), and the forced discharge as a one-shot",
          f"The loop-40 diagram drew 22 Ω permanently across the contactor: it would keep feeding ≈ 1 A into a stopped bus. Now a relay **K_PRE** (10 A / 30 VDC) in series "
          f"with a {R_PRE} Ω RH-50 closes only when both mushrooms are released, the watchdog is healthy and reset is pressed; the main contactor closes when the bus reaches "
          f"V_supply − 3 V; K_PRE then opens. Every stop opens K_PRE too (it hangs on the same safety chain), and it opens after {T_PRE_TIMEOUT:.0f} s if the bus never comes up (latched fault). "
          f"With {pc['C'] * 1e3:.1f} mF on the robot bus (4.7 mF + 15 drivers × 0.47 mF, ESTIMATE): τ = {pc['tau'] * 1e3:.0f} ms, 95 % in {pc['t95'] * 1e3:.0f} ms, inrush {pc['i_in']:.1f} A, "
          f"{pc['E']:.1f} J in the resistor; idle driver draw sags the bus only {pc['droop']:.1f} V. A shorted bus for the full timeout puts {pc['e_fault']:.0f} J in it (rating {pc['e_rating']:.0f} J). "
          "22 Ω would have left the bus ≈ 13 V short with the drivers' idle current.", "",
          f"**Fable M2 (pre-charge fought the clamp):** the earlier text forced the clamp on *while* the contactor was open, and pre-charge runs *while* it is open, so "
          f"{R_PRE} Ω would have fed {pc['R_clamp']:.2f} Ω and the bus would sit at {pc['v_fight']:.1f} V until K_PRE timed out. Now the forced discharge is a "
          f"**{T_FORCE * 1e3:.0f} ms one-shot on the contactor's opening edge** (second SN74LVC1G123, triggered by the aux contact opening), **ANDed with NOT K_PRE**; "
          f"the overvoltage comparator stays a level (it never fires below {CLAMP['v_on'] * (1 - CLAMP['v_tol']) + CLAMP['hyst']:.2f} V, so not during pre-charge). {T_FORCE * 1e3:.0f} ms is {pc['n_tau_force']:.0f} × the "
          f"{pc['tau_dis'] * 1e3:.1f} ms discharge time constant (bus < 0.1 V, {pc['E_dis']:.1f} J into the clamp). Anything that recharges the bus later "
          f"(a falling arm back-driving its motors) is held under {CLAMP['v_on'] * (1 + CLAMP['v_tol']) + CLAMP['hyst']:.1f} V by the comparator and bled by a permanent {R_BLEED / 1000:.0f} kΩ resistor "
          f"(≈ {pc['t_bleed']:.0f} s to 10 V; {pc['i_bleed'] * 1000:.0f} mA, {pc['droop_bleed']:.2f} V of pre-charge droop). Sequence and netlist: *Safety board* below.", "",
          "### Watchdog timing budget (SW2 < 50 ms)",
          f"The control loop runs at {WATCHDOG['loop_hz']:.0f} Hz and toggles the heartbeat line only after a complete healthy cycle. The watchdog is an SN74LVC1G123 "
          f"retriggerable monostable (t_w ≈ R·C, TI SLVA720) on its own 3.3 V LDO, set to **{WATCHDOG['t_nom'] * 1e3:.0f} ms** (±{WATCHDOG['tol']:.0%} with a C0G capacitor; "
          "sw/config/bringup.yaml `watchdog_s`, the one value the board, the virtual bench and the bring-up check use). "
          "Its output gates the MOSFET in series with the contactor coil, which also runs through both mushrooms' NC contacts. "
          "**The contactor alone isn't torque-off:** the drives keep running from the bus capacitors. The contactor's aux contact opening fires the discharge one-shot, "
          "so the bus is discharged through the clamp resistors.", ""] + watchdog_lines(st, st1) + [
          "| Step (robot, Pi hung) | Time (ms) |", "|---|---|"]
    L += [f"| {n} | {t * 1e3:.1f} |" for n, t in st["rows"]]
    L += [f"| **Total: heartbeat lost → no drive torque** | **{st['total'] * 1e3:.1f}** (M1 bench {st1['total'] * 1e3:.1f}; SW2 budget {SW2_BUDGET * 1e3:.0f}) "
          f"{ok(st['total'] <= SW2_BUDGET and st1['total'] <= SW2_BUDGET)} |", "",
          f"Mushroom pressed: {st['t_button'] * 1e3:.0f} ms. Without the forced discharge, the drivers' idle draw alone would take ≈ {st['t_dis_passive'] * 1e3:.0f} ms to bring the bus to UVLO: "
          "the old design could not meet 50 ms. The software path is faster when the Pi is alive: the supervisor latches, sends DISABLE to every drive the next cycle "
          f"(≤ {st['t_sw'] * 1e3:.0f} ms) and stops the heartbeat. Day-1 checks: the drives' UVLO and CAN-timeout behaviour, the ALEV100 release time with its coil suppression; "
          "measure button → contacts → current → motion with a supported dummy load, including a hung Pi.", ""] + bench_supply_lines(d) + safety_board_lines() + [
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


def pack_check(cont, peak):
    """Neon review #2 finding 11: pack, BMS, main fuse and the software pack-current budget against the branch currents."""
    r_pack = 6 * PACK["dcir_cell"] / PACK["cells_p"] + R_PATH
    sw = SW_PACK
    return dict(r_pack=r_pack, sag=sw["i_peak"] * r_pack, v50=6 * PACK["ocv50"] - sw["i_peak"] * r_pack, v20=6 * 3.50 - sw["i_peak"] * r_pack,
                v_sag_peak=6 * PACK["ocv50"] - peak * r_pack, trim=1 - sw["i_peak"] / peak)


def pack_lines(cont, peak):
    ok = lambda b: "✅" if b else "❌"
    pc, P, B, sw, F = pack_check(cont, peak), PACK, BMS, SW_PACK, MAIN_FUSE
    return ["### Pack and BMS (Neon review #2 finding 11)",
            f"**Verified.** The BOM paired a {P['name']} (an FPV pack: no BMS, no cell fusing; {P['cont']:.0f} A continuous only with an 80 °C cut-off, "
            f"{P['cont_nocut']:.0f} A without, {P['burst']:.0f} A burst; {P['src']}) with a “40 A JK class” BMS (ESTIMATE). The branch table gives "
            f"{cont:.1f} A worst sustained and ≈ {peak:.0f} A peak: the 40 A BMS trips on the peak and the pack's own continuous rating is below the sustained figure. "
            "The review's alternatives do not fit this bus: a 24 V LiFePO4 pack is 8S (29.2 V full) and a 7S Li-ion pack is 29.4 V full, both over the drives' 28 V "
            "maximum (GIM6010 manual 12–28 V). So the pack stays 6S, gets a BMS with real headroom, and the software caps what it may draw:", "",
            "| Item | Choice | Rating | Load | |", "|---|---|---|---|---|",
            f"| BMS | {B['name']} (${B['price']:.0f}; {B['src']}) | {B['cont']:.0f} A cont. / {B['peak']:.0f} A peak | {peak:.0f} A peak | {ok(B['cont'] >= 2 * peak)} ({B['cont'] / peak:.1f}× on continuous) |",
            f"| Pack, peak | {P['name']} | {P['burst']:.0f} A burst | software cap **{sw['i_peak']:.0f} A** (the {peak:.0f} A diversity peak trimmed {pc['trim']:.0%}, arms first) | {ok(sw['i_peak'] <= 0.8 * P['burst'])} ({P['burst'] / sw['i_peak']:.2f}×) |",
            f"| Pack, sustained | same | {P['cont']:.0f} A with an 80 °C cut-off | software cap **{sw['i_avg']:.0f} A** over a {sw['t_avg']:.0f} s rolling mean; BMS NTCs on the cells, discharge cut at {sw['t_cell_max']:.0f} °C | {ok(sw['i_avg'] <= 0.8 * P['cont'])} |",
            f"| Main fuse (pack → contactor) | {F['name']} ({F['src']}) | {F['rating']} A | ≥ 1.25 × {cont:.1f} A = {1.25 * cont:.0f} A; the {sw['i_peak']:.0f} A cap is {sw['i_peak'] / F['rating']:.0%} for ≤ 1 s | {ok(F['rating'] >= 1.25 * cont)} |",
            f"| Pack connector | XT90 (Amass: 40 A cont. / 90 A burst) instead of the pack's XT60 (30 / 60 A) | 40 A | {sw['i_avg']:.0f} A mean, {sw['i_peak']:.0f} A peak | {ok(sw['i_avg'] <= 40 and sw['i_peak'] <= 90)} |",
            f"| Voltage at the {sw['i_peak']:.0f} A cap (6S2P, {P['dcir_cell'] * 1e3:.0f} mΩ/cell + {R_PATH * 1e3:.0f} mΩ path, ESTIMATE: {pc['r_pack'] * 1e3:.0f} mΩ) | – | drives 12–28 V | "
            f"{pc['v50']:.1f} V at 50 % SoC, {pc['v20']:.1f} V at 20 % | {ok(pc['v20'] >= 12)} (the review is right that it sags under 20 V; it trips nothing) |", "",
            "**Software budget (requirement for sw/, owner of the supervisor):** every cycle the supervisor sums the drives' estimated bus currents "
            f"(the gim_bus_current model above, at the measured bus voltage) and scales the arm torque limits so the sum stays ≤ {sw['i_peak']:.0f} A and the "
            f"{sw['t_avg']:.0f} s mean ≤ {sw['i_avg']:.0f} A; base and lift keep priority (they hold the robot up). The BMS reports current and cell temperatures on CAN A, so the "
            "budget is checked against a measurement, not only the model. Typical draw is far below either cap (next paragraph).", "",
            f"Packaging: the pack sits in a steel battery box in the base bay (ESTIMATE line in the BOM), the BMS's two NTCs taped to the cells; "
            "if the measured mean ever approaches the cap, the Lumenier NDAA variant of the same pack is listed at 60 A cont. / 120 A burst (GetFPV; price not found).", ""]


def report(d=BASELINE):
    rows = branches(d)
    cont = sum(r[1] for r in rows)
    peak = sum(r[2] for r in rows) * 0.6                          # not every branch peaks at once (diversity 0.6, ESTIMATE)
    L = ["# Electrical architecture (auto-generated by `python -m rnd.electrical`)", "",
         "## Power distribution (24 V class, 6S Li-ion)", "",
         "```",
         "6S2P 10 Ah pack ─ JK-B2A8S20P BMS (200 A) ─ XT90 ─ MAIN FUSE 60 A ─┬─ LOGIC branch (always on): 5 V 5 A DC-DC → Pi 5, cameras, lidar, CAN/TTL adapters",
         "                                                         └─ E-STOP CONTACTOR (ALEV100, DC-rated; coil: mushroom NC ×2 in series → watchdog MOSFET; aux contact → feedback + discharge)",
         "                                                            └─ PRE-CHARGE branch around it: relay K_PRE (opens on every stop) + 3.3 Ω RH-50 (A5 v2: switched, never permanent)",
         "                                                              ├─ REGEN CLAMP / BUS DISCHARGE on the motor bus: TLV3202 comparator on 26.0 / off 25.7 V ± 1 %, 2 µs filter, 50 µs min on-time → 3 × IRFB7530 → 3 × 0.56 Ω NH-50, each with a freewheel Schottky (Neon #2 F1); forced on by a 100 ms one-shot on the contactor's opening edge (never while K_PRE is closed)",
         "                                                              ├─ 12 V 10 A DC-DC → STS3215 servos (A5: now downstream of the contactor)",
         "                                                              ├─ F 20 A → base: 4 wheel cartridges (CAN bus A)",
         "                                                              ├─ F 10 A → lift cartridge + pawl solenoid (CAN bus A)",
         "                                                              ├─ F 20 A → left arm via detach plug (CAN bus B, TTL bus 1)",
         "                                                              └─ F 20 A → right arm via detach plug (CAN bus C, TTL bus 2)",
         "LOGIC 24 V ─ F 1 A ─ D11 FACE SCREEN (ESP32-S3-Touch-LCD-4.3B, 7–36 V input; CAN A drop at the head; isolated DI senses the e-stop loop)",
         "DOCK PADS (skirt, 45°) ─ ideal-diode reverse-polarity guard ─ F 7.5 A ─ BMS charge port   (charger 25.2 V CC/CV, 5 A)",
         "```", "",
         "**Docking charge path (loop 27):**",
         "- The dock's output is off until the robot answers on the ID pad: the base sensor node pulls a coded load, the dock's ESP32-C3 checks it, then closes its MOSFET.",
         "- The robot may run while it charges (idle ≈ 33 W ≈ 1.4 A of the 5 A), so a stock charger could never see current fall to its end-of-charge level. The robot therefore tells the dock to stop at 4.15 V/cell over the ID line (from BMS telemetry).",
         "- The JK-B2A8S20P is a common-port BMS (charge and load on the same P−); confirm on arrival, the wiring above assumes it.", "",
         "| Branch | Worst sustained bus current (A), for fuse/wire sizing | Peak (A) | Fuse (A, slow-blow) |", "|---|---|---|---|"]
    L += [f"| {n} | {c:.1f} | {p:.1f} | {f} |" for n, c, p, f in rows]
    from .budget import power_budget
    _, _, avg_w, hours = power_budget(d)
    L += [f"| **Pack** | **{cont:.1f}** | **{peak:.0f}** (diversity 0.6) | main {MAIN_FUSE['rating']} |", "",
          f"Typical draw is far lower: the duty-cycle power budget averages **{avg_w:.0f} W ≈ {avg_w / V_NOM:.1f} A** (runtime {hours:.1f} h on 222 Wh).", "",
          ""] + pack_lines(cont, peak) + [
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
          "| Arm detach plug (blind-mate, floats in a printed mount beside the V-band) | **XT60** (Amass 30 A cont. / 60 A burst; was XT30, 15 A, under the 20 A fuse and the 38 A peak: Neon #2 F10) | "
          "**10-pin** 2 mm pogo block: CAN1 H/L, CAN2 H/L, **USB D+/D− (adjacent, GND pins either side)**, 3 × GND, ID strap (the old 6-pin block had no USB at all; rnd/harness.py) |",
          "| Upper ↔ base coupling (under the cam lever) | **3 × XT60**: left arm, right arm, lift + 12 V + logic (one XT60 at 30 A was under the upper's sustained load; rnd/harness.py *Lift umbilical*) | "
          "12-pin pogo: CAN A H/L, e-stop loop ×2 (torso mushroom), ESTOP_SENSE, heartbeat, AUX_OK, 5 V sense, GND ×2, always-on logic 24 V ×2 (≤ 4 A) |",
          "| Battery slide-out | **XT90** (keyed; 40 A cont. / 90 A burst) | balance lead to the charger only |",
          f"| Dock contacts (skirt at 45°, flush brass pads) | + at {DOCK_PADS['+'][0]:.0f} mm, − at {DOCK_PADS['−'][0]:.0f} mm, {DOCK_PAD_W:.0f} × {DOCK_PADS['+'][1]:.0f} mm pads, 5 A (Neon #2 F12: were 60 × 12 mm at 60 / 100) | "
          f"ID pad at {DOCK_PADS['ID'][0]:.0f} mm ({DOCK_PAD_W:.0f} × {DOCK_PADS['ID'][1]:.0f}) → 1 kΩ + 5.6 V TVS → base sensor node (handshake, stop-charge command); dock_sim.md *Contacts and retention* |",
          "| Every joint cartridge | 2-pin JST-VH (24 V) or GH (12 V) | 4-pin JST-GH CAN/TTL daisy chain |", "",
          "**Cable routing (loop 32, rnd/harness.py):** torsion through the short shaft bores can't absorb the joint ranges (cable rated ~±360°/m). "
          "Every joint crossing uses a **clock-spring spiral** in an annular cavity around the joint axis. No slip rings. "
          "D12 keeps the stiffest cable at Ø2.2 mm: the wrist camera is MIPI-CSI on a flat-flex through the wrist, with a USB bridge on the forearm. "
          "Loop 40: yaw back to ±150° with a real clock-spring cassette; **D14:** the elbow node makes 12 V and drives the TTL servo bus, so only 24 V, CAN (two pairs, A14) "
          "and USB cross the shoulder joints. Neon #2 F10: the 24 V pair is 16 AWG thin-wall fluoropolymer (was 20 AWG silicone), the USB crosses the detach plug on its own pogo pair "
          "to a hub in the torso e-tray, and the base → torso lift umbilical is now specified. See [harness.md](harness.md)."]
    return "\n".join(L), cont, peak


if __name__ == "__main__":
    md, c, p = report()
    (DOCS / "electrical.md").write_text(md)
    print(md)
