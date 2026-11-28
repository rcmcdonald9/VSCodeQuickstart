"""Safety-board proof before power (Astra review #3, critical 3): circuit simulation of netlist v3.
python -m rnd.safety_board_sim  → docs/rnd/safety_board_sim.json + .md, docs/rnd/safety_board_tests.md

Method. The analog parts run in **ngspice** (batch mode) when it is installed (`apt-get install ngspice`; this module
found it at the path printed in the report); the board logic (LATCH, watchdog, one-shots, timers, contactor operate /
release) is an event-driven model in python that follows the netlist in electrical.md gate by gate and hands the switch
times to the SPICE decks (two passes where a logic event depends on an analog crossing, e.g. PRE_OK). Without ngspice the
same circuits run as a python ODE model (explicit, 1 µs steps, ideal switches; the report says which ran). Parts:

1. **Regen clamp dump**: the board (bulk + film capacitors, TVS, bleeder, three NH-50 legs with their winding inductance,
   freewheel Schottky, IRFB7530 (VDMOS model), gate resistor, UCC27524 behavioural driver, TLV3202 as a hysteretic switch
   with its delay) and the **harness to every drive** (fuse, connectors, 14 / 16 AWG pairs hot at 60 °C, with loop
   inductance) with each drive's DC-link capacitance and its regenerated power as a current source P(t)/V at the drive.
   Cases: the M1 bench arm and the robot (2 arms + lift), the joint-by-joint deceleration profile from
   rnd/electrical.regen_budget and a rectangular envelope (every joint's kinetic energy at its peak power, 323 J /
   2.77 kW on the robot), at the high threshold corner; ×1.25 power margin. Output: highest voltage **at each drive**
   vs the 28 V limit, drain peaks, resistor energy, the regen power each branch may return.
2. **Pre-charge** into the bus capacitors through K2 + 3.3 Ω with the drives' idle draw, PRE_OK, K1 closing after its
   operate time (making current), K2 opening; the 10 Ω dummy-load timeout case.
3. **Watchdog loss → contactor drop → discharge one-shot**: heartbeat stops, monostable times out (±20 %), coil current
   decays through D1 + Z1 (and the coil driver's drain voltage), contacts open, aux opens, FORCE discharges the bus;
   time to driver UVLO **at the far drive**.
4. **Fuse coordination / wire protection** (python): supply latch (43.8–52 A), branch ATO curves vs each wire's
   thermal response (first-order, hot conductor), per branch and per segment.
5. **Logic single faults** (python model of the netlist, v3 / W13b fixes / v4): reset, brownout / power-on, broken / stuck AUX, welded K1,
   loss of the 12 V clamp supply, watchdog tolerance; the uncommanded K1 drop-out race (Fable #2 M2), the K1 closing bounce
   through the AUX debounce, aux glitches, the pre-charge timeout.
6. **Base channel, category-1 base stop** (W15, docs/sw/base_stop.md): the e-stop sequence with the base on its own switch
   behind a 2.0 s OFF-delay safety relay (board event model + the base's speed: arm torque off, base ramped, base power
   off), the base bus's regen during the ramp in ngspice (pack sinking or not, with and without a base clamp leg), the
   phase-short braking a drive must not do, and whether the D18b caster lock may engage at speed (snag model on the twin's
   mass and inertia).

Everything here is a model of a board that does not exist yet: it decides whether the netlist is worth building and what
the bench must measure (docs/rnd/safety_board_tests.md); it is not evidence that the built board works.
"""
from __future__ import annotations

import json
import math
import shutil
import subprocess
import tempfile
from pathlib import Path

import numpy as np

from . import electrical as el

DOCS = Path(__file__).resolve().parent.parent / "docs" / "rnd"
NGSPICE = shutil.which("ngspice")

# ---- harness / board parasitics (ESTIMATE unless noted)
AWG_R20 = {10: 3.28e-3, 14: 8.29e-3, 16: 13.17e-3, 18: 20.9e-3}   # Ω/m per conductor at 20 °C (copper tables)
HOT = 1 + 0.00393 * 40                     # conductors at 60 °C (harness.md assumption)
L_PAIR = 0.6e-6                             # H/m loop inductance of a 24 V pair (ESTIMATE; 0.4 twisted … 1.0 separated)
R_XT60 = 0.6e-3                             # Ω per connector, both contacts (ESTIMATE; Amass ≈ 0.3 mΩ per contact)
R_ATO = {20: 3.4e-3, 15: 4.9e-3, 10: 8.0e-3}   # Ω cold, ATO blade fuses (ESTIMATE, Littelfuse ATO class)
ESR_DRV = 0.05                              # Ω per driver input capacitor (ESTIMATE)
C1_ESR, C1_ESL = 0.015, 20e-9               # 4.7 mF 35 V electrolytic (ESTIMATE)
C2_ESR, C2_ESL = 0.003, 3e-9                # 10 µF film at the MOSFETs
L_LEAD = 10e-9                              # per lead in the MOSFET / diode loop (3 leads ≈ the 30 nH of electrical.L_LOOP)
R_GATE = 4.7                                # Ω gate resistor per IRFB7530: sets the ≈ 0.3 µs edge (VDMOS model below)
R_DRV = 1.0                                 # UCC27524 output resistance (datasheet ≈ 1 Ω)
T_CMP_RC = 40e-9                            # TLV3202 delay modelled as an RC (≈ 55 ns to the logic threshold)
# Fix found by this simulation: the clamp's own turn-on step at the board node (≈ 146 A × C1's ESR + the film cap's sag ≈ 2 V)
# is larger than the 1.2 V hysteresis window, so the 55 ns comparator turns straight off again and the chopper rings at the
# gate-edge rate (MHz). An RC filter on the divider and a minimum on-time (a one-shot ORed into the gate) bound it.
CMP_FILTER = el.CMP_FILTER                  # s, divider filter
T_ON_MIN = el.T_ON_MIN                      # s, minimum clamp on-time (SN74LVC1G123 one-shot on OV ↑, ORed with OV and FORCE)
L_BENCH_LEAD = 1.5                          # m, bench arm lead 14 AWG (ESTIMATE: board on the bench, arm on its stand)
# K1 coil (TE ALEV100 24 V): ESTIMATEs until measured on the part (board test T4 measures the release)
COIL = dict(R=170.0, L=0.5, i_drop_frac=0.15, t_mech=0.004)   # Ω, H, drop-out at 15 % of the hold current, armature travel s
V_Z1 = 24.0 * 1.05                          # 1N5359B at its +5 % corner, plus its dynamic resistance below
R_Z1 = 3.0
Q4_VDS = {"IRLB8721 (as drawn)": 30.0}      # the coil driver's rating (Infineon datasheet: 30 V)
T_AUX_LAG = 0.0005                          # aux contact opens this long after the main contacts (ESTIMATE; T4 measures)
T_K2_RELEASE = 0.005                        # Omron G5LE release (datasheet ≤ 5 ms)
T_K1_OPERATE = (0.005, 0.025)               # ALEV100 operate time, min (ESTIMATE) / max (datasheet)
PRE_OK_DV = 3.0                             # PRE_OK: VBUS ≥ VIN − 3 V (±1 % of VIN → ±0.25 V)


def wire(awg, m):
    """Round-trip R (Ω, hot) and loop L (H) of a 24 V pair."""
    return 2 * m * AWG_R20[awg] * HOT, m * L_PAIR


def _rows(rb):
    return {r[0]: dict(name=r[1], I=r[2], w=r[3], E=r[4], P=r[5], t=r[6]) for r in rb["rows"]}


def branches(which):
    """Harness from the board's VBUS to every drive: [(branch, [(segment, R, L, node, drivers, joint)])].
    Segment lengths: harness.md (*Harness kit*, *Lift umbilical*); the bench lead is an ESTIMATE."""
    arm_tail = [("16 AWG 0.45 + 0.80 m (pitch spiral) + detach XT60 + 0.10 m", *_add(wire(16, 1.35), (R_XT60, 0)), "R", 1, "shoulder_roll"),
                ("16 AWG 0.35 m (roll spiral + link)", *wire(16, 0.35), "Y", 1, "upper_yaw"),
                ("16 AWG 1.15 m (yaw spiral + upper arm)", *wire(16, 1.15), "E", 2, "elbow"),       # elbow drive + elbow node
                ("16 AWG 0.95 m (elbow spiral + forearm)", *wire(16, 0.95), "W", 1, None)]           # DM-J4310 (wrist: no regen budgeted)
    if which == "M1":
        lead = ("F3 20 A ATO + bench lead 14 AWG 1.5 m", *_add(wire(14, L_BENCH_LEAD), (R_ATO[20], 0)), "T", 1, "shoulder_pitch")
        return [("arm", [lead] + arm_tail)]
    out = []
    for side in ("left arm", "right arm"):
        lead = ("F 20 A ATO + base↔upper XT60 + lift umbilical 14 AWG 1.16 m", *_add(wire(14, 1.16), (R_ATO[20] + R_XT60, 0)), "T", 1, "shoulder_pitch")
        out.append((side, [lead] + arm_tail))
    out.append(("lift", [("F 10 A ATO + umbilical 18 AWG 1.16 m", *_add(wire(18, 1.16), (R_ATO[10], 0)), "L", 1, "lift")]))
    out.append(("base", [("F 20 A ATO + 14 AWG 0.5 m", *_add(wire(14, 0.5), (R_ATO[20], 0)), "B", 4, None)]))
    return out


def _add(a, b):
    return a[0] + b[0], a[1] + b[1]


def regen_profiles(which, envelope=True, scale=1.0):
    """{joint: [(t, P)]} at the drive: envelope = the joint's kinetic energy returned at its peak bus power (rectangular, no
    copper-loss credit: conservative); else the deceleration profile P(t) = τ·ω(t) − I²R of regen_budget."""
    rb = el.regen_budget() if which == "robot" else el.regen_budget(payload=2.0, arms=1, lift=False)
    R = _rows(rb)
    if which == "robot":
        tl = 2 * rb["E_lift"] / rb["P_lift"]
        R["lift"] = dict(E=rb["E_lift"], P=rb["P_lift"], t=tl)
    out = {}
    for j, r in R.items():
        P, E = r["P"] * scale, r["E"] * scale
        if envelope:
            T = E / P
            out[j] = [(0.0, 0.0), (1e-3, P), (T, P), (T + 1e-3, 0.0)]
        else:
            ts = r["t"]
            tw = r["I"] * r["w"] / ts * r["w"] if "I" in r else r["P"]    # τ·ω at the start (lift: a linear ramp, no copper term)
            cu = tw - r["P"]                              # copper loss (constant at peak torque)
            pts = [(0.0, 0.0)] + [(1e-3 + ts * k / 20, max(tw * (1 - k / 20) - cu, 0.0) * scale) for k in range(21)]
            out[j] = pts
    return out, rb


# ---------------------------------------------------------------------------------------------------- SPICE plumbing
def run_spice(deck, vectors, tag, allow_partial=False):
    """Run a deck in ngspice batch mode; → dict of numpy arrays ('time' + each vector)."""
    with tempfile.TemporaryDirectory() as td:
        p = Path(td)
        out = p / "out.txt"
        ctrl = ["", ".control", "set wr_singlescale", "set wr_vecnames", "option numdgt=7", "run",
                f"wrdata {out} " + " ".join(vectors), ".endc", ".end"]
        (p / f"{tag}.cir").write_text(deck + "\n".join(ctrl) + "\n")
        r = subprocess.run([NGSPICE, "-b", f"{tag}.cir"], cwd=td, capture_output=True, text=True, timeout=1800)
        if not out.exists():
            raise RuntimeError(f"ngspice {tag} failed:\n{r.stdout[-2000:]}\n{r.stderr[-2000:]}")
        lines = out.read_text().split("\n")
        head = lines[0].split()
        data = np.loadtxt(lines[1:]) if len(lines) > 2 else np.zeros((0, len(head)))
    d = {"time": data[:, 0]}
    t_stop = _num(next(ln.split()[2] for ln in deck.split("\n") if ln.startswith(".tran")))
    if not len(data) or data[-1, 0] < 0.999 * t_stop:
        if not allow_partial:
            raise RuntimeError(f"ngspice {tag} stopped at {data[-1, 0] if len(data) else 0} s of {t_stop} s:\n{r.stdout[-1500:]}")
        d["aborted_at"] = float(data[-1, 0]) if len(data) else 0.0
        d["why"] = next((ln.strip() for ln in (r.stdout + r.stderr).split("Reference value") [-1].split("\n") if "too small" in ln), "")
    for k, v in enumerate(vectors):
        d[v] = data[:, k + 1]
    return d


def _num(x):
    """A SPICE number ('30m', '1u', '0.4')."""
    x = x.lower().rstrip("s")
    for suf, f in (("meg", 1e6), ("m", 1e-3), ("u", 1e-6), ("n", 1e-9), ("p", 1e-12), ("k", 1e3)):
        if x.endswith(suf):
            return float(x[: -len(suf)]) * f
    return float(x)


MODELS = """
.model IRFB7530 VDMOS(Rd=0.9m Rs=0.4m Vto=3.0 Kp=250 Lambda=0.002 Cgdmax=4n Cgdmin=0.25n Cgs=11n Cjo=2.5n Is=1e-11 Rb=1.5m BV=66 IBV=1m)
.model DSCH D(Is=1.1e-5 N=1.0 Rs=2.5m Cjo=1.6n BV=66 IBV=1m)
.model DTVS D(Is=1e-12 N=1 Rs=0.2 BV=36.7 IBV=1m)
.model DIDEAL D(Is=1e-4 N=1 Rs=2m)
.model D4007 D(Is=7e-9 N=1.8 Rs=0.04 Cjo=15p)
.model DSIG D(Is=1e-14 N=1 Rs=1 Cjo=1p)
.model DZ24 D(Is=1e-12 N=1 Rs={rz} BV={vz} IBV=10m)
.model Q4FET VDMOS(Rd=5m Rs=2m Vto=1.8 Kp=40 Cgdmax=0.6n Cgdmin=0.1n Cgs=1n Cjo=0.4n Is=1e-12 BV={q4bv} IBV=1m)
"""


def _board(v_in, cmp_corner, force_pwl="0 0", k1_pwl=None, k1_closed=True, ic=25.2, fix=True, l_res=None):
    """The board's power stage + comparator as deck lines. cmp_corner: (rise, fall) of the OV comparator at VBUS.
    fix: the comparator input filter (CMP_FILTER) and the minimum on-time (T_ON_MIN) this study adds; False = netlist v3
    as drawn (the TLV3202 straight on the divider)."""
    rise, fall = cmp_corner
    vt, vh = (rise + fall) / 2, (rise - fall) / 2
    L = [f"VPSU psu 0 {v_in + 0.2 if v_in else 0}", "RPSU psu psu1 5m", "DID psu1 vin DIDEAL"]   # LM74700 + FET as a 0.2 V diode (blocks regen)
    if k1_closed:
        L.append("RK1 vin vbus 0.3m")
    else:
        L += [f"VK1C k1c 0 PWL({k1_pwl})", "BK1 vin k1x I = (v(vin) - v(k1x)) * max(min(v(k1c), 1), 0) / 1m", "LK1 k1x vbus 100n", "RK1OFF vin k1x 1e7"]   # contacts + board wiring (ESTIMATE)
    L += [f"C1 vbus c1a 4.7m IC={ic}", f"RC1 c1a c1b {C1_ESR}", f"LC1 c1b 0 {C1_ESL}",
          f"C2 vbus c2a 10u IC={ic}", f"RC2 c2a c2b {C2_ESR}", f"LC2 c2b 0 {C2_ESL}",
          f"RBL vbus 0 {el.R_BLEED}", "D3 0 vbus DTVS"]
    for k in (1, 2, 3):
        L += [f"VIR{k} vbus r{k}a 0", f"RR{k} r{k}a r{k}b {el.CLAMP['r_each']}", f"LR{k} r{k}b d{k} {l_res or el.NH50['L']}",
              f"LD{k} d{k} d{k}m {L_LEAD}", f"M{k} d{k}m g{k} s{k} s{k} IRFB7530", f"LS{k} s{k} 0 {L_LEAD}",
              f"LF{k} d{k} f{k} {L_LEAD}", f"VIF{k} f{k} f{k}b 0", f"DF{k} f{k}b vbus DSCH", f"RG{k} drv g{k} {R_GATE}"]
    if fix:     # divider filter + on-time stretch: a diode-charged timer that holds the gate ≥ T_ON_MIN after each OV pulse
        L += ["RFL vbus vsen 100", f"CFL vsen 0 {CMP_FILTER / 100:.3e}", "SCMP v33 cmpr vsen 0 SWH",
              "BTON tchg 0 V = 3.4 * 0.5 * (1 + tanh(20 * (v(cmp) - 1.65)))", "DTON tchg tmr DSIG", f"CTON tmr 0 1n",
              f"RTON tmr 0 {T_ON_MIN / (1e-9 * math.log(3.0 / 1.65)):.0f}"]
        gate = "max(max(v(cmp), v(tmr)), v(force))"
    else:
        L += ["SCMP v33 cmpr vbus 0 SWH"]
        gate = "max(v(cmp), v(force))"
    L += ["V33 v33 0 3.3", "RPD cmpr 0 1k", "RDL cmpr cmp 1k", f"CDL cmp 0 {T_CMP_RC / 1e3}",
          f"VFORCE force 0 PWL({force_pwl})", "V12 v12 0 12",
          f"BDRV drvi 0 V = v(v12) * 0.5 * (1 + tanh(20 * ({gate} - 1.65)))", f"RDRV drvi drv {R_DRV}",
          f".model SWH sw(vt={vt:.4f} vh={vh:.4f} ron=1 roff=1e9)", ".model SWK1 sw(vt=0.5 vh=0.1 ron=1m roff=1e7)"]
    return L


def _harness(which, prof, ic=None, idle_scale=1.0):
    """Branch segments and drive nodes; each drive's input capacitance and idle draw (a resistor drawing I_IDLE_DRIVER at
    25.2 V); regen current sources P(t)/V at the drives."""
    L, nodes = [], {}
    for bi, (bname, segs) in enumerate(branches(which)):
        prev = "vbus"
        for si, (seg, R, Lh, node, n_drv, joint) in enumerate(segs):
            nd = f"n{bi}{node}"
            L += [f"RH{bi}{si} {prev} h{bi}{si} {R:.6f}", f"LH{bi}{si} h{bi}{si} {nd} {max(Lh, 1e-9):.3e}",
                  f"CD{bi}{si} {nd} cd{bi}{si} {n_drv * el.C_DRIVER:.4e} IC={ic or 0}", f"RCD{bi}{si} cd{bi}{si} 0 {ESR_DRV / n_drv:.4f}",
                  f"RQ{bi}{si} {nd} 0 {el.V_FULL / (n_drv * el.I_IDLE_DRIVER * idle_scale):.2f}"]
            if joint and joint in prof:
                pw = " ".join(f"{t:.6f} {p:.3f}" for t, p in prof[joint])
                L += [f"VP{bi}{si} p{bi}{si} 0 PWL({pw})", f"BRG{bi}{si} 0 {nd} I = v(p{bi}{si}) / max(v({nd}), 5)"]
            nodes[nd] = dict(branch=bname, segment=seg, node=node, joint=joint, drivers=n_drv)
            prev = nd
    return L, nodes


VARIANTS = {
    # netlist v3 as drawn: TLV3202 straight on the divider, on at 26.4 V + 0.6, off at 26.4 − 0.6 (electrical.clamp_check's corners)
    "v3": dict(rise=26.4 + 0.6, fall=26.4 - 0.6, fix=False),
    # v3 thresholds + the divider filter and the minimum on-time
    "v3+ton": dict(rise=26.4 + 0.6, fall=26.4 - 0.6, fix=True),
    # proposed (now electrical.CLAMP): filter + on-time, on at 26.0 V, off at 25.7 V (the on-time sets the real swing)
    "fix": dict(rise=el.CLAMP["v_on"] + el.CLAMP["hyst"], fall=el.CLAMP["v_on"] - el.CLAMP["hyst"], fix=True),
    # Fable #2 minor: the proposed board with stock wirewound RH-50s (≈ 5 µH, ESTIMATE) instead of the NH-50s, freewheel diodes kept
    "rh50": dict(rise=el.CLAMP["v_on"] + el.CLAMP["hyst"], fall=el.CLAMP["v_on"] - el.CLAMP["hyst"], fix=True, l_res=el.RH50["L"]),
}
TOL = el.CLAMP["v_tol"]


def corners(variant="v3"):
    v = VARIANTS[variant]
    r, f = v["rise"], v["fall"]
    return dict(high=(r * (1 + TOL), f * (1 + TOL)), nominal=(r, f), low=(r * (1 - TOL), f * (1 - TOL)))


# ---------------------------------------------------------------------------------------------------- 1. clamp dump
def clamp_dump(which="robot", envelope=True, scale=1.0, corner="high", v_in=el.V_FULL, t_extra=0.02, variant="fix", t_end=None):
    prof, rb = regen_profiles(which, envelope, scale)
    t_end = t_end or max(p[-1][0] for p in prof.values()) + t_extra
    hl, nodes = _harness(which, prof, v_in)
    deck = [f"* clamp dump {which} envelope={envelope} scale={scale} corner={corner} variant={variant}", MODELS.format(rz=R_Z1, vz=V_Z1, q4bv=33)]
    fix = VARIANTS[variant]["fix"]
    deck += _board(v_in, corners(variant)[corner], ic=v_in, fix=fix, l_res=VARIANTS[variant].get("l_res")) + hl
    deck += [".options method=gear reltol=2e-3 abstol=1e-6 vntol=1e-4 itl4=100 chgtol=1e-14",
             f".ic v(vbus)={v_in} " + " ".join(f"v({n})={v_in}" for n in nodes),
             f".tran 1u {t_end:.4f} 0 2u uic"]
    vec = ["v(vbus)", "v(d1)", "v(drv)", "i(VIR1)", "i(VIR2)", "i(VIR3)", "i(VIF1)"] + [f"v({n})" for n in nodes]
    if NGSPICE:
        d = run_spice("\n".join(deck), vec, f"dump_{which}", allow_partial=not fix)
        method = "ngspice"
    else:
        d = _ode_dump(which, prof, nodes, corners(variant)[corner], v_in, t_end)
        method = "python ODE"
    t = d["time"]
    on = t > min(1e-3, 0.5 * t[-1])
    e_res = [float(np.trapezoid(d[f"i(VIR{k})"] ** 2 * el.CLAMP["r_each"], t)) for k in (1, 2, 3)]
    edges = int(np.sum(np.diff((d["v(drv)"] > 6).astype(int)) == 1))
    drive = {n: float(d[f"v({n})"][on].max()) for n in nodes}
    worst = max(drive, key=drive.get)
    per_node = [prof[nodes[n]["joint"]] for n in nodes if nodes[n]["joint"] in prof]
    e_in = sum(float(np.trapezoid([p for _, p in pr], [t_ for t_, _ in pr])) for pr in per_node)
    t_run = float(d.get("aborted_at", t[-1]))
    return dict(which=which, envelope=envelope, scale=scale, corner=corner, variant=variant, method=method, t_end=t_end, t_run=t_run,
                aborted=d.get("aborted_at") is not None, why=d.get("why", ""), rise_fall=corners(variant)[corner],
                v_drive_min=float(min(d[f"v({n})"][t > 3e-3].min() for n in nodes)) if (t > 3e-3).any() else None,
                regen_P=sum(max(p for _, p in pr) for pr in per_node), regen_E=e_in,
                v_board_max=float(d["v(vbus)"][on].max()), v_board_min=float(d["v(vbus)"][t > 3e-3].min()) if (t > 3e-3).any() else None,
                v_drive_max=drive[worst], worst_node=nodes[worst] | {"id": worst}, drives={n: round(v, 3) for n, v in drive.items()},
                v_drain_max=float(d["v(d1)"].max()), i_leg_max=float(np.max(np.abs(d["i(VIR1)"]))),
                i_fw_max=float(np.max(np.abs(d["i(VIF1)"]))), e_res_each=max(e_res), e_res_total=sum(e_res), edges=edges,
                f_sw_mean=edges / max(t_run - _t_first_edge(t, d["v(drv)"]), 1e-6), nodes=nodes)


def _t_first_edge(t, drv):
    on = drv > 6
    return float(t[np.argmax(on)]) if on.any() else float(t[-1])


def _ode_dump(which, prof, nodes, corner, v_in, t_end, dt=1e-6):
    """Fallback without ngspice: board + harness as an RC/RL network, ideal hysteretic clamp switch (no MOSFET edges, so
    the drain overshoot is not computed: it reports the bus)."""
    rise, fall = corner
    segs = [s for _, ss in branches(which) for s in ss]
    names = list(nodes)
    par = {}
    for bi, (bname, ss) in enumerate(branches(which)):
        prev = None
        for si, s in enumerate(ss):
            par[f"n{bi}{s[3]}"] = prev
            prev = f"n{bi}{s[3]}"
    R = {n: s[1] for n, s in zip(names, segs)}
    Lh = {n: max(s[2], 1e-9) for n, s in zip(names, segs)}
    C = {n: s[4] * el.C_DRIVER for n, s in zip(names, segs)}
    V = {n: v_in for n in names}
    I = {n: 0.0 for n in names}
    vb, on = v_in, False
    Cb = el.C_BULK + 10e-6
    Rcl = el.CLAMP["r_each"] / 3
    ts = np.arange(0, t_end, dt)
    out = {k: np.zeros(len(ts)) for k in ["v(vbus)", "v(d1)", "v(drv)", "i(VIR1)", "i(VIR2)", "i(VIR3)", "i(VIF1)"] + [f"v({n})" for n in names]}
    joint = {n: nodes[n]["joint"] for n in names}
    for k, t in enumerate(ts):
        on = vb > rise or (on and vb > fall)
        i_cl = vb / Rcl if on else 0.0
        i_in = max((v_in - vb) / 0.007, 0.0)
        dvb = (i_in - i_cl - vb / el.R_BLEED + sum(I[n] for n in names if par[n] is None)) / Cb
        for n in names:
            up = vb if par[n] is None else V[par[n]]
            I[n] += dt * (V[n] - up - I[n] * R[n]) / Lh[n]          # I[n]: current from the drive node towards the board
        for n in names:
            pj = prof.get(joint[n])
            p = float(np.interp(t, [a for a, _ in pj], [b for _, b in pj])) if pj else 0.0
            i_down = sum(I[m] for m in names if par[m] == n)
            V[n] += dt * (p / max(V[n], 5) + i_down - I[n] - nodes[n]["drivers"] * el.I_IDLE_DRIVER) / C[n]
        vb += dt * dvb
        out["v(vbus)"][k], out["v(d1)"][k], out["v(drv)"][k] = vb, vb, 12.0 * on
        for j in (1, 2, 3):
            out[f"i(VIR{j})"][k] = i_cl / 3
        for n in names:
            out[f"v({n})"][k] = V[n]
    out["time"] = ts
    return out


def regen_limit(dumps, which, v_lim=el.V_DRIVE_MAX - 0.5):
    """Regen power the bus may take with every drive ≤ v_lim: interpolated over the simulated power scales (proposed board,
    high corner, envelope), since the drive-end voltage is the board ceiling plus a harness drop linear in the current."""
    pts = sorted((d["regen_P"], d["v_drive_max"]) for d in dumps if d["which"] == which and d["variant"] == "fix"
                 and d["corner"] == "high" and d["envelope"])
    P, V = np.array([p for p, _ in pts]), np.array([v for _, v in pts])
    if V.min() > v_lim:
        return dict(P_max=0.0, pts=pts)
    if V.max() <= v_lim:
        return dict(P_max=float(P.max()), pts=pts, beyond=True)
    return dict(P_max=float(np.interp(v_lim, V, P)), pts=pts)


# ---------------------------------------------------------------------------------------------------- 2. pre-charge
def precharge_sim(which="M1", t_operate=T_K1_OPERATE[1], idle_scale=1.0, dummy=None, v_in=el.V_FULL):
    """K2 closes at t = 0 (LATCH ↑); PRE_OK when VBUS ≥ VIN − 3 V; K1 closes t_operate later; K2 opens T_K2_RELEASE after
    the aux. Two passes: the first finds the PRE_OK crossing, the second runs the sequence. dummy: Ω across VBUS (fault test)."""
    t_end = 0.4 if dummy is None else el.T_PRE_TIMEOUT + 0.05

    def deck(k1_close, k2_open):
        hl, nodes = _harness(which, {}, 0, idle_scale=idle_scale)
        k1 = f"0 0 {k1_close:.6f} 0 {k1_close + 5e-5:.6f} 1" if k1_close else "0 0"
        k2 = f"0 0 5e-5 1 {k2_open:.6f} 1 {k2_open + 5e-5:.6f} 0" if k2_open else "0 0 5e-5 1"
        L = [f"* pre-charge {which}", MODELS.format(rz=R_Z1, vz=V_Z1, q4bv=33)] + _board(v_in, corners("fix")["low"], k1_closed=False, k1_pwl=k1, ic=0) + hl
        L += [f"VK2C k2c 0 PWL({k2})", "BK2 vin k2a I = (v(vin) - v(k2a)) * max(min(v(k2c), 1), 0) / 20m", "RK2OFF vin k2a 1e7", f"RPRE k2a k2b {el.R_PRE}", f"LPRE k2b vbus {el.RH50['L']}",
              ".model SWK2 sw(vt=0.5 vh=0.1 ron=20m roff=1e7)"]
        if dummy:
            L.append(f"RDUM vbus 0 {dummy}")
        L += [".options method=gear reltol=1e-3 abstol=1e-6 vntol=1e-5 itl4=100",
              ".ic v(vbus)=0 " + " ".join(f"v({n})=0" for n in nodes), f".tran 10u {t_end} 0 20u uic"]
        return "\n".join(L), nodes

    vec = ["v(vbus)", "v(vin)", "i(VPSU)"]
    if not NGSPICE:
        return _ode_precharge(which, t_operate, idle_scale, dummy, v_in, t_end)
    d1 = run_spice(deck(None, None)[0], vec, "pre1")
    ok = d1["v(vbus)"] >= d1["v(vin)"] - PRE_OK_DV
    t_ok = float(d1["time"][np.argmax(ok)]) if ok.any() else None
    if t_ok is None:                                 # never charged: PRE_TIMEOUT clears LATCH → K2 opens at 1 s
        e_r3 = float(np.trapezoid((d1["v(vin)"] - d1["v(vbus)"]) ** 2 / el.R_PRE, d1["time"]))
        return dict(which=which, t_pre_ok=None, v_final=float(d1["v(vbus)"][-1]), shortfall=float(d1["v(vin)"][-1] - d1["v(vbus)"][-1]),
                    i_inrush=float(np.max(-d1["i(VPSU)"])), e_r3=e_r3, timeout=True, method="ngspice", dummy=dummy, idle_scale=idle_scale)
    t_k1 = t_ok + t_operate
    d2 = run_spice(deck(t_k1, t_k1 + T_AUX_LAG + T_K2_RELEASE)[0], vec, "pre2")
    t = d2["time"]
    i = -d2["i(VPSU)"]
    pre = t < t_k1 - 1e-6
    post = (t >= t_k1) & (t < t_k1 + 0.01)
    dv = float(np.interp(t_k1, t, d2["v(vin)"] - d2["v(vbus)"]))
    e_r3 = float(np.trapezoid(np.where(pre, (d2["v(vin)"] - d2["v(vbus)"]) ** 2 / el.R_PRE, 0.0), t))
    return dict(which=which, t_pre_ok=t_ok, t_k1=t_k1, t_operate=t_operate, dv_at_close=dv, i_inrush=float(i[pre].max()),
                i_make=float(i[post].max()), e_r3=e_r3, v_final=float(d2["v(vbus)"][-1]), timeout=False, idle_scale=idle_scale,
                method="ngspice", C=el.c_bus(which))


def _ode_precharge(which, t_operate, idle_scale, dummy, v_in, t_end):
    C = el.c_bus(which)
    i_idle = el.N_DRIVERS[which] * el.I_IDLE_DRIVER * idle_scale
    dt, v, t, t_ok, e = 1e-5, 0.0, 0.0, None, 0.0
    while t < t_end:
        i = (v_in - v) / el.R_PRE
        e += i * i * el.R_PRE * dt
        load = i_idle * v / v_in + (v / dummy if dummy else 0.0) + v / el.R_BLEED
        v += dt * (i - load) / C
        t += dt
        if t_ok is None and v >= v_in - PRE_OK_DV:
            t_ok = t
            break
    if t_ok is None:
        return dict(which=which, t_pre_ok=None, v_final=v, shortfall=v_in - v, i_inrush=v_in / el.R_PRE, e_r3=e, timeout=True, method="python ODE",
                    dummy=dummy, idle_scale=idle_scale)
    v_close = v_in - (v_in - v) * math.exp(-t_operate / (el.R_PRE * C))
    return dict(which=which, t_pre_ok=t_ok, t_k1=t_ok + t_operate, t_operate=t_operate, dv_at_close=v_in - v_close, i_inrush=v_in / el.R_PRE,
                i_make=(v_in - v_close) / 0.02, e_r3=e, v_final=v_in, timeout=False, idle_scale=idle_scale, method="python ODE", C=C)


# ---------------------------------------------------------------------------------------------------- 3. watchdog → drop → discharge
def coil_drop(suppression="D1 + Z1", q4_bv=33.0):
    """K1 coil turn-off at t = 1 ms (WD_OK low): coil current, Q4 drain voltage, time to drop-out."""
    v = el.V_FULL
    sup = ["D1 coiln dz D4007", "DZ1 coilp dz DZ24"] if suppression == "D1 + Z1" else ["D1 coiln coilp D4007"]
    L = ["* K1 coil drop", MODELS.format(rz=R_Z1, vz=V_Z1, q4bv=q4_bv), f"VL coilp 0 {v}",
         f"RC coilp cm {COIL['R']}", f"LC cm coiln {COIL['L']} IC={v / COIL['R']:.5f}", *sup,
         "MQ4 coiln g4 0 0 Q4FET", "VG g4x 0 PWL(0 3.3 1m 3.3 1.0001m 0)", "RG4 g4x g4 100",
         ".options method=gear reltol=1e-4", ".tran 1u 30m 0 2u uic"]
    if not NGSPICE:
        i0 = v / COIL["R"]
        vclamp = (V_Z1 + 0.8) if suppression == "D1 + Z1" else 0.8
        # di/dt = −(vclamp + i·R)/L  →  i(t) = (i0 + vclamp/R)·e^(−tR/L) − vclamp/R
        tdrop = COIL["L"] / COIL["R"] * math.log((i0 + vclamp / COIL["R"]) / (COIL["i_drop_frac"] * i0 + vclamp / COIL["R"]))
        return dict(suppression=suppression, i_hold=i0, t_dropout=tdrop, v_drain_max=v + vclamp, method="python ODE",
                    t_release=tdrop + COIL["t_mech"])
    d = run_spice("\n".join(L), ["v(coiln)", "i(LC)"], "coil")
    t, il = d["time"], d["i(LC)"]
    i0 = float(il[t < 0.9e-3].mean())
    below = (t > 1e-3) & (il < COIL["i_drop_frac"] * i0)
    tdrop = float(t[np.argmax(below)] - 1e-3) if below.any() else None
    return dict(suppression=suppression, i_hold=i0, t_dropout=tdrop, v_drain_max=float(d["v(coiln)"].max()), method="ngspice",
                t_release=None if tdrop is None else tdrop + COIL["t_mech"])


def discharge_sim(which="robot", t_open=0.0, t_force=el.T_FORCE, v0=el.V_FULL, hold_a=3.0):
    """K1 open (t = 0), aux T_AUX_LAG later fires FORCE for t_force; the drives keep drawing idle current + hold_a (a
    loaded arm still commanded to hold) until their UVLO. → time to UVLO at the board and at the farthest drive."""
    hl, nodes = _harness(which, {}, v0)
    far = list(nodes)[-1] if which == "M1" else [n for n in nodes if nodes[n]["branch"] == "left arm"][-1]
    n0 = list(nodes)[0]
    hl += [f"BHOLD {n0} 0 I = {hold_a} * min(max(v({n0}) - {el.V_UVLO - 0.5}, 0), 1)"]   # a drive still holding torque, until its UVLO
    f0 = T_AUX_LAG
    force = f"0 0 {f0:.6f} 0 {f0 + 1e-7:.7f} 3.3 {f0 + t_force:.6f} 3.3 {f0 + t_force + 1e-7:.7f} 0"
    L = [f"* discharge {which}", MODELS.format(rz=R_Z1, vz=V_Z1, q4bv=33)] + _board(v0, corners("fix")["high"], force_pwl=force, k1_closed=False,
                                                                                       k1_pwl="0 0", ic=v0) + hl
    L += [".options method=gear reltol=1e-3 abstol=1e-6 vntol=1e-5 itl4=100",
          f".ic v(vbus)={v0} " + " ".join(f"v({n})={v0}" for n in nodes), ".tran 1u 30m 0 5u uic"]
    if not NGSPICE:
        C = el.c_bus(which)
        R = el.CLAMP["r_each"] / 3
        t_b = f0 + R * C * math.log(v0 / el.V_UVLO)
        return dict(which=which, t_uvlo_board=t_b, t_uvlo_far=t_b * 1.3, far=far, method="python ODE", i_peak=v0 / R)
    d = run_spice("\n".join(L), ["v(vbus)", f"v({far})", "i(VIR1)"], f"dis_{which}")
    t = d["time"]

    def cross(v):
        b = v < el.V_UVLO
        return float(t[np.argmax(b)]) if b.any() else None
    return dict(which=which, t_uvlo_board=cross(d["v(vbus)"]), t_uvlo_far=cross(d[f"v({far})"]), far=far, far_desc=nodes[far],
                i_peak=3 * float(np.max(np.abs(d["i(VIR1)"]))), v_after_force=float(d["v(vbus)"][-1]), method="ngspice")


def stop_chain(which, coil, dis):
    """Heartbeat lost → no torque at worst tolerances: last edge → monostable +20 % → coil drop-out + armature → aux →
    FORCE → far drive below UVLO → phase current gone."""
    w = el.WATCHDOG
    rows = [("heartbeat lost just after an edge: one 2 ms cycle + 25 ms monostable at +20 %", 1 / w["loop_hz"] + w["t_nom"] * (1 + w["tol"])),
            (f"coil current to drop-out ({coil['suppression']}, coil {COIL['R']:.0f} Ω / {COIL['L']} H ESTIMATE) + armature {COIL['t_mech'] * 1e3:.0f} ms",
             coil["t_release"]),
            ("aux contact lags the mains (ESTIMATE) → FORCE → far drive below UVLO (simulated)", dis["t_uvlo_far"] or math.nan),
            ("AUX debounce, opening → AUX_DB low at the slow corner (v4)", el.aux_debounce()["t_fall"][1]),
            ("phase current decays once the drive stops switching (ESTIMATE)", 0.001)]
    datasheet = el.CONTACTOR["t_release"]
    rows_ds = list(rows)
    rows_ds[1] = ("contactor release, datasheet maximum (suppression unspecified)", datasheet)
    tot = sum(t for _, t in rows)
    tot_ds = sum(t for _, t in rows_ds)
    return dict(rows=rows, total=tot, total_datasheet=tot_ds, rows_datasheet=rows_ds, budget=el.SW2_BUDGET,
                wd_max_for_budget=(el.SW2_BUDGET - (max(tot, tot_ds) - w["t_nom"] * (1 + w["tol"]))) / (1 + w["tol"]))


# ---------------------------------------------------------------------------------------------------- 4. fuse / wire
ATO_MAX = [(1.10, 360000.0), (1.35, 600.0), (2.00, 5.0), (3.50, 0.5), (6.0, 0.02)]    # Littelfuse ATO: max opening time (s)
ATO_MIN = [(1.35, 0.75), (2.00, 0.15), (3.50, 0.08), (6.0, 0.005)]
# wire installations: G (W/m/K per conductor to the surroundings) and C (J/m/K), ΔT limit for a fault and for service
WIRES = {
    "14 AWG silicone, free air (bench lead, umbilical)": dict(awg=14, G=0.14, C=12.0, t_amb=40, t_max=150, t_svc=105),
    "16 AWG ETFE in a clock-spring cassette (ASA)": dict(awg=16, G=0.12, C=7.0, t_amb=40, t_max=85, t_svc=60),
    "18 AWG silicone, drag chain (lift)": dict(awg=18, G=0.10, C=5.0, t_amb=40, t_max=150, t_svc=105),
    "10 AWG silicone, supply → board": dict(awg=10, G=0.20, C=30.0, t_amb=30, t_max=150, t_svc=105),
}


def _loglog(curve, x):
    xs, ys = np.log([a for a, _ in curve]), np.log([b for _, b in curve])
    if x <= curve[0][0]:
        return math.inf
    return float(np.exp(np.interp(math.log(x), xs, ys)))


def wire_temp(w, i, t):
    """Conductor temperature after t s at current i (A), first order with R(T); from t_amb."""
    r20 = AWG_R20[w["awg"]]
    T, dt, tt = w["t_amb"], max(t / 400, 1e-3), 0.0
    while tt < t:
        p = i * i * r20 * (1 + 0.00393 * (T - 20))
        T += dt * (p - w["G"] * (T - w["t_amb"])) / w["C"]
        tt += dt
        if T > 400:
            break
    return T


def wire_ss(w, i):
    r20 = AWG_R20[w["awg"]]
    a = i * i * r20
    den = w["G"] - a * 0.00393
    return math.inf if den <= 0 else w["t_amb"] + a * (1 + 0.00393 * (w["t_amb"] - 20)) / den


def fuse_coordination():
    """Per branch: what opens first at each fault current, and the wire's temperature when it does."""
    psu_lo, psu_hi = (el.BENCH_PSU["i_max"] * f for f in el.BENCH_PSU["trip"])
    cases = [
        ("bench: SE-1000-24 → F1 60 A → board → F3 20 A → 14 AWG arm lead", 20, "14 AWG silicone, free air (bench lead, umbilical)", True),
        ("bench Step 2 / robot: F3 20 A → arm 16 AWG in the clock springs", 20, "16 AWG ETFE in a clock-spring cassette (ASA)", True),
        ("robot: F 20 A → 16 AWG in the clock springs (pack + BMS, no supply latch)", 20, "16 AWG ETFE in a clock-spring cassette (ASA)", False),
        ("proposed: F3 15 A at the board for the arm branch", 15, "16 AWG ETFE in a clock-spring cassette (ASA)", False),
        ("robot: F 10 A → lift 18 AWG", 10, "18 AWG silicone, drag chain (lift)", False),
    ]
    out = []
    for name, rating, wname, bench in cases:
        w = WIRES[wname]
        rows = []
        for I in sorted({round(1.1 * rating, 1), round(1.35 * rating, 1), 25.0, 30.0, 40.0, 43.0, 52.0, 80.0, 150.0}):
            if bench and I >= psu_lo:
                t_open, who = 0.01, f"supply latch ({psu_lo:.1f}–{psu_hi:.1f} A)"
            else:
                t_open, who = _loglog(ATO_MAX, I / rating), f"fuse {rating} A (max curve)"
            T = wire_ss(w, I) if math.isinf(t_open) else wire_temp(w, I, t_open)
            rows.append(dict(I=round(I, 1), opens=who, t_open_s=t_open, T_wire=round(T, 1), ok=T <= w["t_max"]))
        i_svc = next((i / 10 for i in range(10, 1000) if wire_ss(w, i / 10) > w["t_svc"]), None)
        i_fault = next((i / 10 for i in range(10, 1000) if wire_ss(w, i / 10) > w["t_max"]), None)
        unprot = [r for r in rows if not r["ok"]]
        out.append(dict(case=name, rating=rating, wire=wname, rows=rows, i_continuous_service=i_svc, i_continuous_fault=i_fault,
                        protected=not unprot, gap=[r["I"] for r in unprot], bench=bench))
    return out


# ---------------------------------------------------------------------------------------------------- 5. logic model
# K1 aux closing bounce used by the logic model (electrical.K1_BOUNCE: burst 3 ms, longest closed interval 1 ms, ESTIMATE):
# open gaps (s after first contact). Closed 0.3, open 0.3, closed 1.0, open 0.2, closed 0.5, open 0.2, closed 0.3, open 0.2 ms.
BOUNCE_GAPS = [(0.3e-3, 0.6e-3), (1.6e-3, 1.8e-3), (2.3e-3, 2.5e-3), (2.8e-3, 3.0e-3)]
assert abs(BOUNCE_GAPS[-1][1] - el.K1_BOUNCE["burst"]) < 1e-9
assert max(b - a for a, b in zip([0.0] + [g[1] for g in BOUNCE_GAPS], [g[0] for g in BOUNCE_GAPS])) <= el.K1_BOUNCE["closed_max"] + 1e-9


def _db_corner(name):
    """AUX debounce (electrical.AUX_DEBOUNCE) at a corner, with a self-consistent threshold pair (VT+ − VT− ≥ ΔVT min):
    'fast_rise' = lowest VT+ and short RC (rejects the least bounce at closing), 'fast_fall' = highest VT− and short RC
    (rejects the shortest glitch while running), 'slow' = highest VT+, lowest VT−, long RC (slowest FORCE, latest AUX_DB
    at ARM), 'nom'."""
    D = el.AUX_DEBOUNCE
    tc, td = D["r_c"] * D["c"], D["r_d"] * D["r_c"] / (D["r_d"] + D["r_c"]) * D["c"]
    (p0, p1), (n0, n1), h = D["vt_pos"], D["vt_neg"], D["hyst_min"]
    vp, vn, k = dict(fast_rise=(p0, p0 - h, 1 - D["tol"]), fast_fall=(n1 + h, n1, 1 - D["tol"]), slow=(p1, n0, 1 + D["tol"]),
                     nom=((p0 + p1) / 2, (n0 + n1) / 2, 1.0))[name]
    return dict(tau_c=tc * k, tau_d=td * k, vt_pos=vp, vt_neg=vn, vcc=D["vcc"], v_floor=D["v_floor"], name=name)


class Board:
    """Event-driven model of the netlist's logic and relays (DT steps, 20 µs by default). Inputs: mushrooms S1/S2 (pressed =
    open), ARM button, heartbeat edges, TH1, logic supply; faults. Signals follow electrical.md's netlist. Variants:
    'v3' as drawn; 'fix' = v3 + the W13b changes (¬AUX on the ARM condition, discrepancy monitor 30 ms, 12 V UVLO);
    'v4' = fix + the Fable #2 M2 race fix: AUX debounced (AUX_DB, electrical.AUX_DEBOUNCE at a tolerance corner), a RUN
    latch so K2_DRIVE = LATCH ∧ ¬RUN ∧ ¬PRE_TIMEOUT, the discrepancy window at electrical.DISC_WINDOW.
    U6 (SN74LVC1G123) is modelled with its function table: a pulse on Ā ↓ (B high, CLR̄ high) and on CLR̄ ↑ (Ā low, B
    high). On an AUX edge that also moves CLR̄ low (K2_DRIVE following AUX: v3 / fix) the trigger and the clear arrive on
    the same edge two gate delays apart: logged as 'FORCE race' and resolved as the repo's model always did (no FORCE),
    because a ns pulse into U10's CLR̄ cannot be relied on either way."""

    def __init__(self, which="M1", fix=False, wd_scale=1.0, faults=(), base=False, ss1_scale=1.0, variant=None, dt=2e-5,
                 bounce=None, db_corner="slow", disc_scale=1.0):
        self.variant = variant or ("fix" if fix else "v3")
        self.fix, self.v4 = self.variant != "v3", self.variant == "v4"
        self.DT = dt
        self.which, self.faults = which, set(faults)
        # AUX bounce at every K1 closing (K1_BOUNCE ESTIMATE), on by default with the v4 board (the debounce is what it tests)
        self.bounce = self.v4 if bounce is None else bounce
        self.db = _db_corner(db_corner)
        self.v_db, self.aux_db = 0.0, False
        self.run = False                               # U28 (v4)
        self.disc_win = (el.DISC_WINDOW["t"] if self.v4 else 0.030) * disc_scale
        self.k1_close_t = None
        # base channel (robot board, §6): own watchdog WD_B, U20 OFF-delay safety relay, Q9 base switch, FORCE_B
        self.base = base
        self.t_ss1 = el.BASE_SS1["t_set"] * ss1_scale
        self.last_hb_b = None
        self.u20_in = self.u20_out = False
        self.u20_off_t = None
        self.base_on = False
        self.force_b_until = -1.0
        self.disc_b_t = None
        self.base_log = []
        self.t = 0.0
        self.wd_t = el.WATCHDOG["t_nom"] * wd_scale
        self.s_pressed = False
        self.th1 = True
        self.logic_on = True
        self.latch = False
        self.last_hb = None
        self.k1_coil = self.k2_coil = False
        self.q4 = False
        self.k1 = self.k2 = False                      # contacts
        self.k1_t = self.k2_t = None                   # time of the last coil change
        self.aux_prev = False
        self.aux_raw_prev = False
        self.k2_sig = False                            # K2_DRIVE as U6's CLR̄ last saw it
        self.force_until = -1.0
        self.pre_t0 = None
        self.pre_timeout = False
        self.vbus = 0.0
        self.vin = el.V_FULL
        self.k1_cmd_off_t = None
        self.disc_t = None
        self.log = []
        self.events = []
        self.por_until = 0.1                           # R9·C9 power-on reset holds CLR̄ low

    # signals
    @property
    def aux_raw(self):
        """AUX_RAW (U12 opto output, TP8): the aux contact, with the closing bounce when enabled."""
        if "aux_broken" in self.faults:
            return False
        if "aux_stuck" in self.faults:
            return True
        if not self.k1:
            return False
        if self.bounce and self.k1_close_t is not None:
            dt = self.t - self.k1_close_t
            for a, b in BOUNCE_GAPS:
                if a <= dt < b:
                    return False
        return self.k1

    @property
    def aux(self):
        """The AUX the logic uses: AUX_DB on v4, AUX_RAW on v3 / fix."""
        return self.aux_db if self.v4 else self.aux_raw

    @property
    def wd_ok(self):
        return self.logic_on and self.last_hb is not None and self.t - self.last_hb <= self.wd_t

    @property
    def wd_b_ok(self):
        return (not self.base) or (self.logic_on and self.last_hb_b is not None and self.t - self.last_hb_b <= self.wd_t)

    @property
    def vbus_b(self):
        """Base bus present: Q9 on (soft-started), or a shorted Q9 (fault q9_short) with VIN up."""
        return self.base_on or "q9_short" in self.faults

    @property
    def pre_ok(self):
        return self.vbus >= self.vin - PRE_OK_DV

    @property
    def force(self):
        return self.t < self.force_until and "no_12v" not in self.faults

    def heartbeat(self, arm=True, base=True):
        if arm:
            self.last_hb = self.t
        if base:
            self.last_hb_b = self.t

    def press_arm(self):
        if not self.logic_on:
            return False
        d = (not self.s_pressed) and self.wd_ok and self.wd_b_ok and self.th1 and (not self.fix or not self.aux)
        clr = self.t < self.por_until
        if d and not clr:
            self.latch = True
            self.pre_t0 = self.t
            self.pre_timeout = False
            self.events.append((self.t, "LATCH set"))
        return self.latch

    def power(self, on):
        self.logic_on = on
        if not on:
            self.latch = False
        else:
            self.por_until = self.t + 0.1
            self.last_hb = None

    def step(self, n=1):
        for _ in range(n):
            self._step()

    def run_for(self, secs):
        self.step(int(round(secs / self.DT)))

    def _k2_drive(self):
        x = self.run if self.v4 else self.aux
        return self.logic_on and self.latch and (not x) and (not self.pre_timeout)

    def _fire(self, t, why):
        if self.force_until < t:
            self.events.append((t, why))
        self.force_until = t + el.T_FORCE

    def _step(self):
        self.t += self.DT
        t, dt = self.t, self.DT
        stop = self.s_pressed or not self.wd_ok or not self.th1 or not self.logic_on or (self.fix and "no_12v" in self.faults)   # fix 4: 12 V UVLO
        stop = stop or not self.wd_b_ok                       # base channel: a lost base heartbeat is a board STOP too
        if self.latch and (stop or self.pre_timeout or t < self.por_until or self.force):
            self.latch = False
            self.events.append((t, "LATCH cleared"))
        if not self.latch:
            self.run = False                                  # U28 CLR̄ = LATCH
        q4 = self.logic_on and self.latch and self.wd_ok and (self.pre_ok or self.aux)
        self.q4 = q4
        coil1 = q4 and not self.s_pressed and "coil_open" not in self.faults
        k2_drive = self._k2_drive()
        coil2 = k2_drive and not self.s_pressed
        if coil1 != self.k1_coil:
            self.k1_coil, self.k1_t = coil1, t
            if not coil1:
                self.k1_cmd_off_t = t
        if coil2 != self.k2_coil:
            self.k2_coil, self.k2_t = coil2, t
        if "k1_welded" not in self.faults:
            if self.k1_coil and not self.k1 and t - self.k1_t >= T_K1_OPERATE[1]:
                self.k1, self.k1_close_t = True, t
                self.events.append((t, "K1 closed"))
            if not self.k1_coil and self.k1 and t - self.k1_t >= el.CONTACTOR["t_release"]:
                self.k1 = False
                self.events.append((t, "K1 open"))
        elif self.k1_coil and not self.k1 and t - self.k1_t >= T_K1_OPERATE[1]:
            self.k1, self.k1_close_t = True, t
        if self.k2_coil and not self.k2 and t - self.k2_t >= 0.010:
            self.k2 = True
            self.events.append((t, "K2 closed"))
        if not self.k2_coil and self.k2 and t - self.k2_t >= T_K2_RELEASE:
            self.k2 = False
        raw = self.aux_raw
        if self.aux_raw_prev and not raw:
            self.events.append((t, "AUX_RAW open"))
        self.aux_raw_prev = raw
        if self.v4:                                           # U29: asymmetric RC into a Schmitt buffer (exact exponential per step)
            D = self.db
            if raw:
                self.v_db = D["vcc"] - (D["vcc"] - self.v_db) * math.exp(-dt / D["tau_c"])
            else:
                self.v_db = D["v_floor"] + max(self.v_db - D["v_floor"], 0.0) * math.exp(-dt / D["tau_d"])
            if not self.aux_db and self.v_db >= D["vt_pos"]:
                self.aux_db = True
            elif self.aux_db and self.v_db <= D["vt_neg"]:
                self.aux_db = False
        aux = self.aux
        if self.v4 and aux and not self.aux_prev and self.latch:
            self.run = True                                   # U28: CLK = AUX_DB ↑, D = 3V3
        k2_now = self._k2_drive()                             # U6 CLR̄ = ¬K2_DRIVE as it settles after this edge
        if self.aux_prev and not aux:                         # U6: Ā falling edge
            if not k2_now:
                self._fire(t, "FORCE")
            elif self.latch:
                self.events.append((t, "RACE: U6 triggered and cleared by the same AUX edge (K2_DRIVE rises)"))
        elif self.k2_sig and not k2_now and not aux:          # U6: CLR̄ ↑ with Ā low, B high (SN74LVC1G123 function table)
            self._fire(t, "FORCE (U6 CLR̄ ↑: pre-charge aborted)")
        self.k2_sig = k2_now
        if self.fix:                                          # K1 discrepancy monitor: coil command (Q4 gate) ≠ AUX for > the window
            cmd = self.q4                                     # the netlist XORs the Q4 gate (a coil with no current still reads "commanded")
            if cmd != aux:
                self.disc_t = t if self.disc_t is None else self.disc_t
                if t - self.disc_t > self.disc_win and self.force_until < t:
                    self._fire(t, "FORCE (K1 discrepancy)")
                    self.latch = False
                    self.disc_t = None
            else:
                self.disc_t = None
        self.aux_prev = aux
        opened = self.run if self.v4 else aux
        if self.latch and self.pre_t0 is not None and not opened and t - self.pre_t0 > el.T_PRE_TIMEOUT:
            self.pre_timeout = True
        # bus (lumped RC): the supply through K1 (stiff) or the pre-charge resistor, the clamp when forced or over its
        # threshold, the drives' idle draw and the bleeder
        r_cl = el.CLAMP["r_each"] / 3
        clamp = (self.force or self.vbus > corners("fix")["nominal"][0]) and "no_12v" not in self.faults
        if self.k1:
            self.vbus = self.vin * (r_cl / (r_cl + 0.02) if clamp else 1.0)
        else:
            i = (self.vin - self.vbus) / el.R_PRE if self.k2 else 0.0
            i -= (self.vbus / r_cl if clamp else 0.0) + (self.vbus / 10.0 if "dummy10" in self.faults else 0.0) + el.N_DRIVERS[self.which] * el.I_IDLE_DRIVER * self.vbus / el.V_FULL + self.vbus / el.R_BLEED
            self.vbus = max(self.vbus + dt * i / el.c_bus(self.which), 0.0)
        self.log.append((t, self.latch, self.k1, self.k2, self.force, self.vbus))
        if self.base:
            self._base_step(t)

    def _base_step(self, t):
        """U20 (G9SA): input = mushroom channels c/d closed ∧ LATCH (photoMOS in channel 1); its OFF-delay contacts feed Q9's
        enable, so they open t_ss1 after the input drops and close again only with the input back (ARM). FORCE_B: a 100 ms
        one-shot on the enable's falling edge. DISC_B: base bus present > 50 ms with the enable off (shorted Q9)."""
        u_in = (not self.s_pressed) and self.latch and self.logic_on
        if self.u20_in and not u_in:
            self.u20_off_t = t
            self.events.append((t, "U20 input off (SS1 delay starts)"))
        if u_in:
            self.u20_out, self.u20_off_t = True, None
        elif self.u20_out and self.u20_off_t is not None and t - self.u20_off_t >= self.t_ss1:
            self.u20_out = False
        self.u20_in = u_in
        on = self.u20_out
        if self.base_on and not on:
            self.force_b_until = t + el.T_FORCE
            self.events.append((t, "base switch off"))
            self.events.append((t, "FORCE_B"))
        elif on and not self.base_on:
            self.events.append((t, "base switch on"))
        self.base_on = on
        if self.vbus_b and not on:
            if self.disc_b_t is None:
                self.disc_b_t, self.disc_b_fired = t, False
            if t - self.disc_b_t > 0.050 and not self.disc_b_fired:
                self.disc_b_fired = True
                self.events.append((t, "DISC_B"))
        else:
            self.disc_b_t = None
        self.base_log.append((t, u_in, self.u20_out, self.base_on, t < self.force_b_until))


def _run_hb(b, secs, hb=True, period=0.002, hb_b=None):
    hb_b = hb if hb_b is None else hb_b
    n = int(round(secs / b.DT))
    k = int(round(period / b.DT))
    for i in range(n):
        if (hb or hb_b) and i % k == 0:
            b.heartbeat(arm=hb, base=hb_b)
        b.step()


VARIANT_TAG = {"v3": "v3 as drawn", "fix": "v3 + fixes (W13b)", "v4": "v4 (race fix + AUX debounce)"}
T7_LIMIT = 0.004            # s: T7, far end < 10 V after AUX opens


def logic_scenarios(t_far=0.0015):
    """Each scenario on the netlist as drawn (v3), with the W13b fixes, and with the v4 race fix + AUX debounce (worst-case
    'slow' debounce corner unless the row says otherwise; v4 boards see the K1 closing bounce at every ARM). t_far: FORCE
    on → far drive below UVLO (§3 discharge, M1), for T7's 'far end < 10 V after AUX'."""
    res = []

    def arm_and_run(b):
        _run_hb(b, 0.15)
        b.press_arm()
        _run_hb(b, 0.3)
        return b

    def after(b, t0, pre):
        return [t for t, e in b.events if e.startswith(pre) and t > t0]

    for var in ("v3", "fix", "v4"):
        tag = VARIANT_TAG[var]
        B = lambda **kw: Board(variant=var, **kw)
        # 1. normal arm → run → watchdog loss → stop, no self re-arm
        b = arm_and_run(B())
        ok_run = b.k1 and not b.k2
        t0 = b.t
        _run_hb(b, 0.3, hb=False)
        k1_open = next((t for t, e in b.events if e == "K1 open" and t > t0), None)
        forced = bool(after(b, t0, "FORCE"))
        _run_hb(b, 0.5)                        # heartbeat back: must stay off until ARM
        res.append(dict(case="heartbeat stops while running", variant=tag, ok=ok_run and k1_open is not None and forced and not b.k1 and not b.latch,
                        detail=f"K1 open {1e3 * (k1_open - t0):.1f} ms after the last edge (wd {b.wd_t * 1e3:.0f} ms + release), FORCE {'yes' if forced else 'NO'}, "
                               f"heartbeat back → K1 {'closed (self re-arm!)' if b.k1 else 'stays open'}"))
        # 2. ARM refused with a mushroom pressed, or without a heartbeat
        b = B()
        _run_hb(b, 0.15)
        b.s_pressed = True
        r1 = b.press_arm()
        b.s_pressed = False
        b2 = B()
        b2.run_for(0.15)
        r2 = b2.press_arm()
        res.append(dict(case="ARM with a mushroom pressed / without heartbeat", variant=tag, ok=not r1 and not r2, detail=f"LATCH {r1} / {r2}"))
        # 3. brownout: logic supply dips (supply trip) and returns
        b = arm_and_run(B())
        b.power(False)
        _run_hb(b, 0.05, hb=False)
        b.power(True)
        _run_hb(b, 0.5)
        res.append(dict(case="logic supply lost and restored (PSU trip, brownout)", variant=tag, ok=not b.k1 and not b.latch,
                        detail=f"after power returns: LATCH {b.latch}, K1 {'closed' if b.k1 else 'open'} (power-on reset)"))
        # 4. mushroom tapped while armed
        b = arm_and_run(B())
        b.s_pressed = True
        b.run_for(0.0005)
        b.s_pressed = False
        _run_hb(b, 0.4)
        res.append(dict(case="mushroom tapped (coil loop broken for 0.5 ms) while armed", variant=tag, ok=not b.k1 and not b.latch,
                        detail=f"LATCH {b.latch}, K1 {'closed' if b.k1 else 'open'}"))
        # 5. broken AUX wire while running
        b = arm_and_run(B())
        b.faults.add("aux_broken")
        t0 = b.t
        _run_hb(b, 0.3)
        f = after(b, t0, "FORCE")
        k1o = next((t for t, e in b.events if e == "K1 open" and t > t0), None)
        k2c = after(b, t0, "K2 closed")
        res.append(dict(case="AUX wire breaks while running", variant=tag, ok=not b.k1, t_force=(f[0] - t0) if f else None,
                        detail=(f"FORCE at +{1e3 * (f[0] - t0):.1f} ms ({'the AUX_DB edge' if var == 'v4' else 'discrepancy monitor'}) and LATCH cleared, K1 open at "
                                f"+{1e3 * (k1o - t0):.0f} ms{'; K2 re-closes at +%.0f ms first (harmless here: K1 is still closed)' % (1e3 * (k2c[0] - t0)) if k2c else ''} "
                                "(the clamp loads the supply for the release time: the bench supply may latch); stop, fail-safe")
                        if f and k1o else "no FORCE (U6 is held cleared: K2_DRIVE = LATCH ∧ ¬AUX goes high and K2 re-closes), K1 stays closed on PRE_OK: "
                                          "a latent fault; the next stop gets **no forced discharge** (≈ 300 ms of bus on the drives' idle draw). "
                                          "Only the Pi sees it (AUX_OK low → supervisor e-stop latch)"))
        # 6. AUX stuck closed (welded aux, shorted opto) before an ARM
        b = B(faults=("aux_stuck",))
        _run_hb(b, 0.15)
        b.press_arm()
        _run_hb(b, 0.1)
        vb_at_close = next((b.log[i - 1][5] for i, lg in enumerate(b.log) if lg[2] and i), None)
        res.append(dict(case="AUX stuck closed, then ARM", variant=tag, ok=not b.latch,
                        detail=("K1 closes on an empty bus (pre-charge bypassed: Q4 holds on AUX, K2_DRIVE = ¬AUX is off), "
                                f"bus {0.0 if vb_at_close is None else vb_at_close:.1f} V at closing → inrush limited only by ESR / wiring; "
                                "and no aux edge at the next stop, so no FORCE") if b.latch else "ARM refused (¬AUX on the ARM condition)"))
        # 7. K1 welded: stop requested
        b = arm_and_run(B())
        b.faults.add("k1_welded")
        t0 = b.t
        _run_hb(b, 0.3, hb=False)
        f = after(b, t0, "FORCE")
        q4_off = b.k1_cmd_off_t
        res.append(dict(case="K1 main contacts welded, heartbeat stops", variant=tag, ok=bool(f) and (f[0] - q4_off) <= el.SW2_BUDGET,
                        t_q4_force=(f[0] - q4_off) if f else None,
                        detail=(f"FORCE {1e3 * (f[0] - t0):.0f} ms after the last edge ({1e3 * (f[0] - q4_off):.0f} ms after Q4 off, ≤ {el.SW2_BUDGET * 1e3:.0f}): the clamp loads the supply at "
                                f"{el.V_FULL / (el.CLAMP['r_each'] / 3):.0f} A → the bench supply latches off (≥ 43.8 A); on the robot the pack "
                                "keeps the bus up: needs a second break (BMS discharge FET off)") if f else
                               "no aux edge → no FORCE; the bus stays powered from the supply; only the Pi (DISABLE frames) and the drives' CAN timeout remain"))
        # 8. 12 V clamp supply lost (U2 dead) while running and regenerating
        b = arm_and_run(B(faults=("no_12v",)))
        res.append(dict(case="12 V gate-driver supply lost (U2 / F2) while running", variant=tag, ok=not b.k1,
                        detail=("12 V UVLO → STOP: K1 opens, LATCH cleared (the bus then has no clamp: the 1 kΩ bleeder and the SMDJ33A only)" if var != "v3" else
                                "the comparator still switches but UCC27524 has no supply: no clamp, no FORCE, and K1 stays closed. Regen then charges the bus "
                                f"without limit (M1 1.3 kW into 7 mF: 25 → 28 V in ≈ {1e3 * 7.05e-3 * (28 ** 2 - 25.2 ** 2) / 2 / 1328:.0f} ms)")))
        # 9. K1 drops out uncommanded: coil unplugged while running (T7's last stimulus, T9 b), then plugged back in
        for case, t_open in (("K1 coil unplugged while running (T7, T9 b), replugged after 0.25 s", 0.25),
                             ("K1 coil supply dips for 20 ms (coil wire intermittent, shock)", 0.020)):
            b = arm_and_run(B())
            t0 = b.t
            b.faults.add("coil_open")
            _run_hb(b, t_open)
            b.faults.discard("coil_open")
            _run_hb(b, 0.4)
            ta = after(b, t0, "AUX_RAW open")
            f = after(b, t0, "FORCE")
            race = after(b, t0, "RACE")
            k2c = after(b, t0, "K2 closed")
            k1c = after(b, t0, "K1 closed")
            t7 = (f[0] - ta[0] + t_far) if (f and ta) else None
            ok = bool(f) and t7 is not None and t7 <= T7_LIMIT and not k2c and not k1c and not b.latch
            bits = [f"AUX opens at +{1e3 * (ta[0] - t0):.0f} ms" if ta else "AUX never opens"]
            bits.append(f"FORCE {1e3 * (f[0] - ta[0]):.1f} ms after AUX ({'AUX_DB edge' if var == 'v4' and f[0] - ta[0] < 0.005 else 'discrepancy monitor' if var != 'v3' else 'how?'}) → "
                        f"far end < 10 V at {1e3 * t7:.1f} ms (T7 ≤ {T7_LIMIT * 1e3:.0f})" if f and ta else "**no FORCE**")
            if race:
                bits.append("the AUX edge triggers U6 and clears it in the same edge (race)")
            if k2c:
                bits.append(f"**K2 re-closes at +{1e3 * (k2c[0] - t0):.0f} ms** (pre-charge starts again with LATCH high)")
            if k1c:
                bits.append(f"**K1 re-closes at +{1e3 * (k1c[0] - t0):.0f} ms without ARM**")
            bits.append(f"end: LATCH {b.latch}, K1 {'closed' if b.k1 else 'open'}")
            res.append(dict(case=case, variant=tag, ok=ok, t7=t7, k2_reclose=bool(k2c), detail="; ".join(bits)))
        # 10. K1 closing bounce at ARM (Fable #2 M2: no debounce); v4 at both debounce corners + the shortest discrepancy window
        rows = [("K1 closing with a 3 ms aux bounce (ESTIMATE burst) at ARM", dict(bounce=True))]
        if var == "v4":
            rows = [("K1 closing with a 3 ms aux bounce at ARM, fast-rise debounce corner (least rejection)", dict(bounce=True, db_corner="fast_rise")),
                    (f"K1 closing with a 3 ms aux bounce at ARM, slow debounce corner + discrepancy window at −{el.DISC_WINDOW['tol']:.0%} (nuisance trip)",
                     dict(bounce=True, db_corner="slow", disc_scale=1 - el.DISC_WINDOW["tol"]))]
        for case, kw in rows:
            b = B(**kw)
            _run_hb(b, 0.15)
            t0 = b.t
            b.press_arm()
            _run_hb(b, 0.3)
            f, race = after(b, t0, "FORCE"), after(b, t0, "RACE")
            k1c = after(b, t0, "K1 closed")
            ok = b.k1 and b.latch and not f and not race
            det = (f"armed (K1 closed at +{1e3 * (k1c[0] - t0):.0f} ms), no FORCE, no race" if ok else
                   f"{len(race)} bounce gaps each trigger U6 and clear it on the same edge (K2_DRIVE = LATCH ∧ ¬AUX follows every gap): a ns FORCE pulse into "
                   "U10's CLR̄ that may or may not clear LATCH, so the board arms only some of the time; if it takes, the clamp sits across a closed K1 and the bench supply latches off"
                   if race else f"FORCE / not armed (LATCH {b.latch}, K1 {b.k1}, FORCE {len(f)})")
            res.append(dict(case=case, variant=tag, ok=ok, detail=det))
        # 11. aux glitch while running (shock / vibration opens the aux for 0.1 ms)
        b = arm_and_run(B(db_corner="fast_fall") if var == "v4" else B())
        t0 = b.t
        b.faults.add("aux_broken")
        _run_hb(b, 0.0001)
        b.faults.discard("aux_broken")
        _run_hb(b, 0.2)
        f, race = after(b, t0, "FORCE"), after(b, t0, "RACE")
        ok = b.k1 and b.latch and not f and not race
        res.append(dict(case="aux contact glitches open for 0.1 ms while running (shock)" + (", fast-fall debounce corner" if var == "v4" else ""), variant=tag, ok=ok,
                        detail=("ignored: still armed, no FORCE (glitches under "
                                f"{1e3 * el.aux_debounce()['gap_reject']:.2f} ms never reach AUX_DB)" if ok else
                                "race on the glitch edge (U6 triggered and cleared together); K2_DRIVE pulses" + ("; the discrepancy monitor then stops the board" if f else ""))))
        # 12. pre-charge never completes (T4: 10 Ω across VBUS): PRE_TIMEOUT, and U6's CLR̄ ↑ trigger
        b = B(faults=("dummy10",))
        _run_hb(b, 0.15)
        t0 = b.t
        b.press_arm()
        _run_hb(b, 1.3)
        f = [(t, e) for t, e in b.events if e.startswith("FORCE") and t > t0]
        k1c = after(b, t0, "K1 closed")
        ok = not k1c and not b.latch and all(t - t0 > el.T_PRE_TIMEOUT for t, _ in f)
        res.append(dict(case="pre-charge into a 10 Ω load (T4): never reaches PRE_OK", variant=tag, ok=ok,
                        detail=f"K1 {'closed!' if k1c else 'never closes'}, PRE_TIMEOUT clears LATCH at {1e3 * (next((t for t, e in b.events if e == 'LATCH cleared' and t > t0), t0) - t0):.0f} ms; "
                               + (f"FORCE once at +{1e3 * (f[0][0] - t0):.0f} ms as K2_DRIVE falls (SN74LVC1G123 CLR̄ ↑ with Ā low): empties the partly charged bus; none during the pre-charge"
                                  if f else "no FORCE")))
    return res


# ---------------------------------------------------------------------------------------------------- 6. base channel (category-1 base stop)
G = 9.81
CRR_HARD = 0.015            # rolling resistance of the omni wheels coasting on a hard floor (ESTIMATE; drive.py: carpet 0.05–0.10)
SNAG_DELTA = 0.003          # m, contact compliance of a locked caster meeting a step edge: printed foot + PTFE pad + guide (ESTIMATE)
ROD = dict(d=0.008, n=2, lever=0.040, sy=350e6)   # D18b guide rods Ø8 h6 (BOM), lever foot contact → lower guide (ESTIMATE), yield (ESTIMATE)
F_ROD = ROD["n"] * ROD["sy"] * math.pi * ROD["d"] ** 3 / 32 / ROD["lever"]     # N horizontal at one foot that yields its two rods
SNAG_FRAC = 0.25            # snag kinetic energy ≤ 25 % of the tip-over energy (a factor 2 on speed)
# SW-T2 (W16a, implemented): a forced lock waits until the base is this slow. Read from the supervisor so this report checks
# the value the software runs (the snag model below allows ≈ 0.28 m/s).
V_CASTER_LOCK = __import__("sw.safety", fromlist=["V_CASTER_LOCK"]).V_CASTER_LOCK
T_SEQ = 6.0                 # s simulated after the trigger (long enough for a coast from 0.8 m/s)


def _sw():
    from sw import safety as s
    return s


def base_state(lift="mid", load=0.0):
    """M, COM, inertia about the COM (MuJoCo twin via rnd.tip.Body) for a base-stop state: arms parked (TUCK)."""
    from . import tip
    from .twin import TUCK
    d = tip.design_after()
    B = tip.Body(d)
    mj, m, dat = B.mj, B.m, B.d
    lid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, "lift")
    q = np.array([B.lift_mid if lift == "mid" else float(m.jnt_range[lid][1])] + [TUCK[j] for j in tip.ARM_J])
    M, c, p = B.com(q, load)
    I = np.zeros((3, 3))
    for b in range(1, m.nbody):
        R = dat.ximat[b].reshape(3, 3)
        r = dat.xipos[b] - c
        I += R @ np.diag(m.body_inertia[b]) @ R.T + m.body_mass[b] * (r @ r * np.eye(3) - np.outer(r, r))
    r = p - c
    I += load * (r @ r * np.eye(3) - np.outer(r, r))
    return dict(M=float(M), c=c, I=I, d=d, lift=lift, load=load)


def caster_snag(states, speeds=(0.12, 0.25, 0.30, 0.50, 0.80)):
    """A locked D18b caster can follow the floor down but not be pushed up (tip.md), so a step edge in its path (a rug edge,
    tile lip, cable, a dip's far side) meets a rigid foot: at speed the base cannot be lifted over it in time and the
    caster becomes a floor-level snag. Model: a dead stop at the leading caster(s), angular momentum about the snag line
    conserved, then the robot pitches about it. Tip energy = raising the COM over that line; force = the horizontal
    impulse over the contact compliance SNAG_DELTA (triangular pulse), shared by the casters on the snag line."""
    s = _sw()
    rows = []
    for st in states:
        d, M, c, I = st["d"], st["M"], st["c"], st["I"]
        h, rc = float(c[2]), d.base.caster_r
        label = f"{'unloaded transit' if st['load'] == 0 else '%.1f kg' % st['load']}, arms parked, lift {st['lift']}"
        for az in range(0, 360, 45):
            u = np.array([math.cos(math.radians(az)), math.sin(math.radians(az)), 0.0])
            lead = [math.cos(math.radians(az - ca)) for ca in (45, 135, 225, 315)]
            xp = rc * max(lead)
            n_snag = sum(1 for x_ in lead if x_ > max(lead) - 1e-6)
            x = xp - float(c[:2] @ u[:2])
            n = np.array([-u[1], u[0], 0.0])
            Ip = float(n @ I @ n) + M * (h * h + x * x)
            rho = math.hypot(h, x)
            pe = M * G * (rho - h)
            a0 = math.atan2(x, h)
            for v in speeds:
                L = M * v * h
                ke = L * L / (2 * Ip)
                theta = a0 - math.acos(min(math.cos(a0) + ke / (M * G * rho), 1.0)) if ke < pe else a0
                J = M * v * (1 - M * h * h / Ip)
                rows.append(dict(state=label, az=az, v=v, n_snag=n_snag, x=x, h=h, M=M, frac=ke / pe, theta_deg=math.degrees(theta), J=J,
                                 F_each=J * v / SNAG_DELTA / n_snag, v_tip=math.sqrt(pe * 2 * Ip) / (M * h),
                                 v_frac=math.sqrt(SNAG_FRAC * pe * 2 * Ip) / (M * h),
                                 v_force=math.sqrt(0.5 * F_ROD * SNAG_DELTA * n_snag * Ip / (M * (Ip - M * h * h)))))
    worst = {}
    for r in rows:                      # per state and speed: the heading with the most tip energy, and the largest force of any heading
        k = (r["state"], r["v"])
        if k not in worst or r["frac"] > worst[k]["frac"]:
            worst[k] = dict(r, F_max=worst.get(k, {}).get("F_max", 0.0))
        worst[k]["F_max"] = max(worst[k]["F_max"], r["F_each"])
    ok_at = lambda r: r["frac"] <= SNAG_FRAC and r["F_each"] <= 0.5 * F_ROD
    v_frac, v_force = min(r["v_frac"] for r in rows), min(r["v_force"] for r in rows)
    return dict(rows=sorted(worst.values(), key=lambda r: (r["state"], r["v"])), F_rod=F_ROD, delta=SNAG_DELTA, frac_lim=SNAG_FRAC,
                v_lock=V_CASTER_LOCK, v_locked_cap=s.CASTER_LOCKED_V, v_frac=v_frac, v_force=v_force, v_allow=min(v_frac, v_force),
                v_tip=min(r["v_tip"] for r in rows),
                lock_ok=all(ok_at(r) for r in rows if r["v"] <= V_CASTER_LOCK + 1e-9),
                locked_cap_ok=all(ok_at(r) for r in rows if r["v"] <= s.CASTER_LOCKED_V + 1e-9),
                fast_ok=all(ok_at(r) for r in rows))


def _base_profile(v0, a, t0):
    """Regen power per kg at the base drives during a ramp from v0 at a (m/s²) starting at t0: P/M = a·v(t). No copper,
    rolling or drivetrain loss is credited (conservative)."""
    T = v0 / a
    return [(0.0, 0.0), (max(t0 - 1e-4, 0.0), 0.0)] + [(t0 + T * k / 40, a * (v0 - a * T * k / 40)) for k in range(41)] + [(t0 + T + 1e-3, 0.0)]


def base_sequence(name, v0, a_sw, M, pi_alive=True, trigger="mushroom", a_qs=None, ss1_scale=1.0, wd_scale=1.2, power_lost=False):
    """E-stop sequence on the robot board with the base channel (event model, 0.1 ms) + the base's speed.
    trigger: 'mushroom' (S1 pressed and held) or 'fault' (the supervisor latches; the arm heartbeat stops at once).
    Pi alive: the supervisor ramps the base at a_sw and keeps the base heartbeat until the base is stopped (sw/safety.py 17).
    Pi hung: both heartbeats stop; the wheel drives' CAN timeout fires and they quick-stop at a_qs while still powered.
    Power lost: everything off at once and the base coasts (drives float their phases: requirement 4)."""
    s = _sw()
    b = Board("robot", variant="v4", base=True, wd_scale=wd_scale, ss1_scale=ss1_scale, dt=1e-4)
    _run_hb(b, 0.15)
    b.press_arm()
    _run_hb(b, 0.4)
    assert b.k1 and b.base_on, "arming failed"
    t0, dt = b.t, b.DT
    k_hb = int(round(1 / el.WATCHDOG["loop_hz"] / dt))
    v, x, latched, t_ramp0, t_stop = v0, 0.0, False, None, None
    lock_prop = None
    hb_a = hb_b = True
    t_sense = 1 / el.WATCHDOG["loop_hz"] if trigger == "mushroom" else 0.0     # ESTOP_SENSE seen on the next cycle
    trace = []
    if power_lost:
        b.power(False)
    for i in range(int(T_SEQ / dt)):
        tt = b.t - t0
        if trigger == "mushroom":
            b.s_pressed = True
        if pi_alive and not power_lost:
            if not latched and tt >= t_sense:
                latched, t_ramp0, hb_a = True, tt, False
            hb_b = (not latched) or v > s.V_STOPPED
            if i % k_hb == 0 and (hb_a or hb_b):
                b.heartbeat(arm=hb_a, base=hb_b)
        powered = b.base_on and not power_lost
        if powered and pi_alive and latched:
            a = a_sw
        elif powered and not pi_alive:
            a = a_qs if tt >= el.DRIVE_QS["can_timeout"] else 0.0
        elif powered:
            a = 0.0
        else:
            a = CRR_HARD * G
        v = max(v - a * dt, 0.0)
        x += v * dt
        if t_stop is None and v == 0.0:
            t_stop = tt
        if lock_prop is None and (v <= V_CASTER_LOCK or not powered):
            lock_prop = (tt, v)
        if i % 50 == 0:
            trace.append((round(tt, 4), round(v, 4), int(b.k1), int(b.base_on)))
        b.step()
    ev = [(t - t0, e) for t, e in b.events if t > t0]
    t_k1 = next((t for t, e in ev if e == "K1 open"), None)
    t_off = next((t for t, e in ev if e == "base switch off"), None)
    v_off = next((vv for tt, vv, _, _ in trace if t_off is not None and tt >= t_off), 0.0)
    return dict(name=name, v0=v0, a_sw=a_sw, a_qs=a_qs, M=M, pi_alive=pi_alive, trigger=trigger, ss1_scale=ss1_scale, power_lost=power_lost,
                t_k1_open=t_k1, t_base_off=t_off, v_at_base_off=v_off, t_ramp_start=t_ramp0, t_stopped=t_stop, x_stop=x,
                lock_sw=lock_prop, E_kin=0.5 * M * v0 ** 2)


def base_regen(M, v0, a, t_ramp0, t_off, sink=True, clamp=True, corner="high", logic_w=None, idle_scale=1.0):
    """ngspice: the robot's VIN (pack + BMS; or the pack with the BMS charge FET off, so it can only discharge) with the
    always-on logic load and its capacitance → Q9 base switch (on until t_off, then only its body diode) → VBUS_B (+ the
    base clamp leg and FORCE_B) → F 20 A + 14 AWG 0.5 m → the 4 wheel drives' 470 µF + idle draw + the ramp's regen P(t)/V."""
    v_in = el.V_FULL
    logic_w = el.LOGIC_MIN_W if logic_w is None else logic_w
    seg, R, Lh, node, n_drv, _ = next(ss for bn, ss in branches("robot") if bn == "base")[0]
    prof = _base_profile(v0, a, t_ramp0)
    t_end = t_off + 0.03
    rise, fall = corners("fix")[corner]
    pk = el.pack_check(1.0, 1.0)["r_pack"]
    L = [f"* base regen sink={sink} clamp={clamp}", MODELS.format(rz=R_Z1, vz=V_Z1, q4bv=33)]
    L += [f"VPACK pack 0 {v_in}", f"RPACK pack vin {pk:.4f}"] if sink else [f"VPACK pack 0 {v_in + 0.2}", "DPACK pack vin DIDEAL"]
    L += [f"CVIN vin cvx {el.C_VIN_ROBOT} IC={v_in}", "RCVIN cvx 0 20m", f"BLOG vin 0 I = {max(logic_w, 1e-6)} / max(v(vin), 10)",
          f"VQ9C q9c 0 PWL(0 1 {t_off:.6f} 1 {t_off + 1e-5:.6f} 0)", "BQ9 vin vbb I = (v(vin) - v(vbb)) * max(min(v(q9c), 1), 0) / 2m",
          "RQ9OFF vin vbb 1e7", "DQ9 vbb vin DIDEAL",
          f"C2B vbb c2ba 10u IC={v_in}", f"RC2B c2ba 0 {C2_ESR}", f"RBLB vbb 0 {el.R_BLEED}",
          f"RHB vbb hb {R:.6f}", f"LHB hb nb {max(Lh, 1e-9):.3e}", f"CDB nb cdb {n_drv * el.C_DRIVER:.4e} IC={v_in}", f"RCDB cdb 0 {ESR_DRV / n_drv:.4f}",
          f"RQB nb 0 {el.V_FULL / (n_drv * el.I_IDLE_DRIVER * idle_scale):.2f}",
          "VPB pb 0 PWL(" + " ".join(f"{t:.6f} {p * M:.3f}" for t, p in prof) + ")", "BRGB 0 nb I = v(pb) / max(v(nb), 5)"]
    vec = ["v(vbb)", "v(nb)", "v(vin)", "i(VPACK)"]
    if clamp:
        f0 = t_off + T_AUX_LAG
        L += ["VIRB vbb rba 0", f"RRB rba rbb {el.CLAMP['r_each']}", f"LRB rbb db {el.NH50['L']}", f"LDB db dbm {L_LEAD}", "MB dbm gb sb sb IRFB7530",
              f"LSB sb 0 {L_LEAD}", f"LFB db fb {L_LEAD}", "VIFB fb fbb 0", "DFB fbb vbb DSCH", f"RGB drvb gb {R_GATE}",
              "RFLB vbb vsenb 100", f"CFLB vsenb 0 {CMP_FILTER / 100:.3e}", "SCMPB v33 cmprb vsenb 0 SWHB",
              "BTONB tchgb 0 V = 3.4 * 0.5 * (1 + tanh(20 * (v(cmpb) - 1.65)))", "DTONB tchgb tmrb DSIG", "CTONB tmrb 0 1n",
              f"RTONB tmrb 0 {T_ON_MIN / (1e-9 * math.log(3.0 / 1.65)):.0f}", "V33 v33 0 3.3", "RPDB cmprb 0 1k", "RDLB cmprb cmpb 1k", f"CDLB cmpb 0 {T_CMP_RC / 1e3}",
              f"VFB forceb 0 PWL(0 0 {f0:.6f} 0 {f0 + 1e-7:.7f} 3.3 {f0 + el.T_FORCE:.6f} 3.3 {f0 + el.T_FORCE + 1e-7:.7f} 0)",
              "BDRVB drvib 0 V = 12 * 0.5 * (1 + tanh(20 * (max(max(v(cmpb), v(tmrb)), v(forceb)) - 1.65)))", f"RDRVB drvib drvb {R_DRV}",
              f".model SWHB sw(vt={(rise + fall) / 2:.4f} vh={(rise - fall) / 2:.4f} ron=1 roff=1e9)"]
        vec += ["i(VIRB)"]
    L += [".options method=gear reltol=2e-3 abstol=1e-6 vntol=1e-4 itl4=100 chgtol=1e-14",
          f".ic v(vbb)={v_in} v(nb)={v_in} v(vin)={v_in}", f".tran 5u {t_end:.4f} 0 10u uic"]
    E_in = M * float(np.trapezoid([p for _, p in prof], [t for t, _ in prof]))
    out = dict(sink=sink, clamp=clamp, corner=corner, M=M, v0=v0, a=a, t_off=t_off, E_regen=E_in, P_peak=M * a * v0, method="ngspice",
               logic_w=logic_w, idle_scale=idle_scale)
    if not NGSPICE:
        C = n_drv * el.C_DRIVER + el.C_VIN_ROBOT
        e_net = 0.0 if sink else max(E_in - logic_w * v0 / a / 2, 0.0)
        vmax = math.sqrt(v_in ** 2 + 2 * e_net / C)
        return out | dict(v_drive_max=min(vmax, rise) if clamp else vmax, v_bus_max=min(vmax, rise) if clamp else vmax,
                          method="python estimate (no ngspice)", t_uvlo_after_off=None, e_leg=None)
    d = run_spice("\n".join(L), vec, f"base_{int(sink)}{int(clamp)}_{logic_w:.0f}")
    t = d["time"]
    before, after = t <= t_off, t > t_off

    def cross(vv):
        b_ = after & (vv < el.V_UVLO)
        return float(t[np.argmax(b_)] - t_off) if b_.any() else None
    e_leg = float(np.trapezoid(d["i(VIRB)"] ** 2 * el.CLAMP["r_each"], t)) if clamp else 0.0
    e_leg_ramp = float(np.trapezoid(np.where(before, d["i(VIRB)"] ** 2 * el.CLAMP["r_each"], 0.0), t)) if clamp else 0.0
    return out | dict(v_drive_max=float(d["v(nb)"][before].max()), v_bus_max=float(d["v(vbb)"][before].max()), v_vin_max=float(d["v(vin)"][before].max()),
                      t_uvlo_after_off=cross(d["v(nb)"]), e_leg=e_leg, e_leg_ramp=e_leg_ramp,
                      i_into_pack=float(max(np.max(-d["i(VPACK)"]), 0.0)) if sink else 0.0, v_nb_end=float(d["v(nb)"][-1]))


def base_logic_scenarios():
    """The base channel's logic on the robot board (U20, WD_B, Q9, FORCE_B, DISC_B): event model, nominal tolerances."""
    res, tag = [], "v4 + base channel (robot)"

    def armed(**kw):
        b = Board("robot", variant="v4", base=True, **kw)
        _run_hb(b, 0.15)
        b.press_arm()
        _run_hb(b, 0.4)
        return b

    def evs(b, t0):
        out = {}
        for t, e in b.events:
            if t > t0 and e not in out:
                out[e] = t - t0
        return out
    t_ss1 = el.BASE_SS1["t_set"]
    # B1 mushroom held, Pi alive (the base heartbeat continues: the supervisor ramps the base)
    b = armed()
    t0 = b.t
    b.s_pressed = True
    _run_hb(b, t_ss1 + 0.3, hb=False, hb_b=True)
    ev = evs(b, t0)
    ok = "K1 open" in ev and "base switch off" in ev and abs(ev["base switch off"] - t_ss1) < 0.01 and "FORCE_B" in ev
    res.append(dict(case="mushroom pressed while running (base channel)", variant=tag, ok=ok,
                    detail=f"K1 open at +{1e3 * ev.get('K1 open', math.nan):.0f} ms (arm category 0); the base switch stays on through the ramp and "
                           f"opens at +{ev.get('base switch off', math.nan):.3f} s (U20 OFF-delay {t_ss1:.1f} s); FORCE_B fires"))
    # B2 base heartbeat lost alone (base loop dead, arm loop alive)
    b = armed()
    t0 = b.t
    _run_hb(b, t_ss1 + 0.3, hb=True, hb_b=False)
    ev = evs(b, t0)
    res.append(dict(case="base heartbeat stops while running (arm loop alive)", variant=tag, ok="K1 open" in ev and "base switch off" in ev,
                    detail=f"WD_B low → STOP: K1 open at +{1e3 * ev.get('K1 open', math.nan):.0f} ms, base switch off at +{ev.get('base switch off', math.nan):.3f} s "
                           "(the drives' CAN timeout ramps the base meanwhile)"))
    # B3 heartbeats come back during the delay: no self re-arm, still off at expiry
    b = armed()
    t0 = b.t
    _run_hb(b, 0.2, hb=False)
    _run_hb(b, t_ss1 + 0.5)
    ev = evs(b, t0)
    res.append(dict(case="heartbeats return during the SS1 delay", variant=tag, ok=(not b.base_on) and (not b.k1) and "base switch off" in ev,
                    detail=f"base switch off at +{ev.get('base switch off', math.nan):.3f} s anyway, K1 open, LATCH {b.latch}: only ARM re-energises U20"))
    # B4 ARM refused without the base heartbeat
    b = Board("robot", variant="v4", base=True)
    _run_hb(b, 0.15, hb=True, hb_b=False)
    r = b.press_arm()
    res.append(dict(case="ARM without the base heartbeat", variant=tag, ok=not r, detail=f"LATCH {r}"))
    # B5 Q9 shorted: the base bus stays after the delay → DISC_B
    b = armed(faults=("q9_short",))
    t0 = b.t
    b.s_pressed = True
    _run_hb(b, t_ss1 + 0.3, hb=False, hb_b=True)
    ev = evs(b, t0)
    res.append(dict(case="Q9 (base switch) shorted, mushroom pressed", variant=tag, ok="DISC_B" in ev,
                    detail=f"enable off at +{ev.get('base switch off', math.nan):.3f} s but the base bus stays: DISC_B at +{ev.get('DISC_B', math.nan):.3f} s → GPIO + "
                           "the BMS discharge FET off over CAN (second break, as for a welded K1); the drives are already disabled / quick-stopped"))
    return res


def base_section():
    """§6: e-stop sequence with the base channel, regen on the base bus, caster lock at speed."""
    s = _sw()
    st_mid, st_top = base_state("mid"), base_state("top")
    M = st_mid["M"]
    a_tr = min(s.CASTER_RELEASED_A, s.Supervisor.base_stop_decel_for(0.0))       # transit, casters released: the limit in force
    a_bk = min(s.A_LOAD, s.Supervisor.base_stop_decel_for(9.07))                  # basket carry: load-mode cap vs the tip table
    v_tr, v_bk = s.BASE_V_MAX, s.CASTER_LOCKED_V
    qs = el.DRIVE_QS
    seqs = [base_sequence("mushroom, Pi alive: transit 0.8 m/s, casters released", v_tr, a_tr, M),
            base_sequence("supervisor fault (e.g. over-temperature), Pi alive: transit", v_tr, a_tr, M, trigger="fault"),
            base_sequence("mushroom, U20 at −10 %", v_tr, a_tr, M, ss1_scale=1 - el.BASE_SS1["tol"]),
            base_sequence("mushroom, U20 at +10 %", v_tr, a_tr, M, ss1_scale=1 + el.BASE_SS1["tol"]),
            base_sequence(f"Pi hung (both heartbeats stop), drive quick-stop written to {qs['a_max']:.1f} m/s²", v_tr, a_tr, M, pi_alive=False, trigger="fault", a_qs=qs["a_max"]),
            base_sequence(f"Pi hung, drive quick-stop at its default {qs['a_default']:.1f} m/s²", v_tr, a_tr, M, pi_alive=False, trigger="fault", a_qs=qs["a_default"]),
            base_sequence(f"Pi hung, default quick-stop, U20 at −10 %", v_tr, a_tr, M, pi_alive=False, trigger="fault", a_qs=qs["a_default"], ss1_scale=1 - el.BASE_SS1["tol"]),
            base_sequence(f"mushroom, R2d basket carry 9.07 kg at {v_bk:.2f} m/s (CASTER_LOCKED_V)", v_bk, a_bk, M + 9.07),
            base_sequence("pack / supply lost (BMS trip): everything off, base coasts", v_tr, a_tr, M, pi_alive=False, trigger="fault", power_lost=True)]
    s0 = seqs[0]
    regen = []
    for sink, clamp, lw, ids in ((True, False, None, 1.0), (False, False, None, 1.0), (False, False, 4.0, 0.5), (False, False, 0.0, 0.5),
                                 (False, True, 0.0, 0.5), (False, True, None, 1.0)):
        regen.append(base_regen(M, v_tr, a_tr, s0["t_ramp_start"], s0["t_base_off"], sink=sink, clamp=clamp, logic_w=lw, idle_scale=ids))
    snag = caster_snag([st_mid, st_top])
    # phase-short braking (requirement 4): back-EMF into the shorted winding, x-drive wheels at 45°
    g = el.MOTOR_CU["GIM6010-8"]
    r_w = st_mid["d"].base.wheel_d / 2
    w = v_tr * math.cos(math.radians(45)) / r_w
    i_sc = g[0] * w / (2 * g[1])                         # output Ke ≈ Kt; line-to-line R = 2 R_phase (ESTIMATE)
    F = 4 * g[0] * i_sc / r_w * math.cos(math.radians(45))
    a_short = min(F / M, 0.5 * G)
    v_short_ok = 0.5 / (F / M / v_tr)
    return dict(M=M, h_mid=float(st_mid["c"][2]), h_top=float(st_top["c"][2]), a_transit=a_tr, a_basket=a_bk, v_basket=v_bk, seqs=seqs, regen=regen, snag=snag,
                phase_short=dict(a=a_short, a_lin=F / M, v_ok=v_short_ok, i=i_sc), ss1=el.BASE_SS1, qs=qs,
                ramp_max=v_tr / a_tr, grace=s.BASE_STOP_GRACE)


# ---------------------------------------------------------------------------------------------------- report
def run(quick=False):
    R = dict(method="ngspice " + (NGSPICE or "") if NGSPICE else "python ODE (ngspice not installed)", date=__import__("time").strftime("%Y-%m-%d"))
    dumps = []
    T = 0.05                       # every joint is at its peak power for the first 42 ms of the envelope: the voltage peaks are inside
    for which in ("M1", "robot"):
        dumps.append(clamp_dump(which, variant="fix", t_end=None if not quick else T))      # full length: resistor energy
        if quick:
            continue
        dumps.append(clamp_dump(which, variant="v3", t_end=0.004))                           # as drawn: rings, aborts
        dumps.append(clamp_dump(which, variant="v3+ton", t_end=T))
        dumps.append(clamp_dump(which, variant="fix", envelope=False, t_end=T))
        for sc in (0.25, 0.45, 1.25):
            dumps.append(clamp_dump(which, variant="fix", scale=sc, t_end=T))
        dumps.append(clamp_dump(which, variant="fix", corner="low", t_end=T))
        if which == "M1":                                                                    # stock RH-50 instead of NH-50 (Fable #2 minor)
            for sc in (0.45, 1.0):
                dumps.append(clamp_dump(which, variant="rh50", scale=sc, t_end=T))
    R["dumps"] = dumps
    R["regen_limit"] = {w: regen_limit(dumps, w) for w in ("M1", "robot")}
    R["precharge"] = [precharge_sim("M1"), precharge_sim("robot"), precharge_sim("M1", t_operate=T_K1_OPERATE[0]),
                      precharge_sim("robot", t_operate=T_K1_OPERATE[0]), precharge_sim("robot", idle_scale=1.5),
                      precharge_sim("M1", dummy=10.0)]
    R["coil"] = [coil_drop("D1 + Z1", q4_bv=33.0), coil_drop("D1 + Z1", q4_bv=100.0), coil_drop("D1 only", q4_bv=100.0)]
    for c, q in zip(R["coil"], ("IRLB8721 (30 V, as drawn)", "100 V part (proposed)", "100 V part (proposed)")):
        c["q4"] = q
    R["discharge"] = [discharge_sim("M1"), discharge_sim("robot")]
    R["stop_chain"] = {w: stop_chain(w, R["coil"][1], d) for w, d in zip(("M1", "robot"), R["discharge"])}
    R["fuses"] = fuse_coordination()
    R["logic"] = logic_scenarios(t_far=R["discharge"][0]["t_uvlo_far"] - T_AUX_LAG) + base_logic_scenarios()
    R["debounce"] = el.aux_debounce()
    R["base"] = base_section()
    for d in R["dumps"]:
        d.pop("nodes", None)
    return R


def _f(x, n=2, unit=""):
    return "–" if x is None or (isinstance(x, float) and not math.isfinite(x)) else f"{x:.{n}f}{unit}"


def write(R):
    (DOCS / "safety_board_sim.json").write_text(json.dumps(R, indent=1, default=float))
    md = report_md(R)
    (DOCS / "safety_board_sim.md").write_text(md)
    (DOCS / "safety_board_tests.md").write_text(tests_md(R))
    return md


def report_md(R):
    L = ["# Safety board simulation (generated by `python -m rnd.safety_board_sim`)", "",
         f"{R['date']}. Method: **{R['method']}** for the analog decks; the board logic is an event-driven python model of the "
         "electrical.md netlist (**v4**: the Fable #2 M2 race fix and the AUX debounce; v3 as drawn and the W13b fixes are kept for comparison). "
         "Astra review #3, critical 3: *simulate the board before power*. A model of an unbuilt board: "
         "it decides what to build and what the bench measures ([safety_board_tests.md](safety_board_tests.md)); it is not evidence the board works.", "",
         "## 1. Regen clamp dump: bus overshoot at the drives", "",
         "Board VBUS (C1 4.7 mF, C2 10 µF film, SMDJ33A, 1 kΩ bleeder, 3 × NH-50 0.56 Ω + 1 µH, VS-40CPQ060 freewheel, IRFB7530 VDMOS, "
         f"R_G {R_GATE} Ω, UCC27524, TLV3202 as a hysteretic switch with its delay) → harness (fuse, XT60s, 14/16 AWG pairs at 60 °C, "
         f"{L_PAIR * 1e6:.1f} µH/m) → each drive's 470 µF + its regen as P(t)/V. Corners: ± 1 % on both thresholds (TL431B + 0.1 % divider). "
         f"Variants: *v3 as drawn* (on {VARIANTS['v3']['rise']:.1f} / off {VARIANTS['v3']['fall']:.1f} V, comparator straight on the divider); *proposed* (on {VARIANTS['fix']['rise']:.1f} / off {VARIANTS['fix']['fall']:.1f} V, {CMP_FILTER * 1e6:.0f} µs divider filter, {T_ON_MIN * 1e6:.0f} µs minimum on-time).", "",
         "| Board | Case | Regen peak / energy (run) | Clamp on / off at VBUS | **Highest drive** (node) | Lowest drive | Drain peak | Leg current | Energy per NH-50 | Chopper f | |",
         "|---|---|---|---|---|---|---|---|---|---|---|"]
    vname = {"v3": "v3 as drawn", "v3+ton": "v3 thresholds + filter + on-time", "fix": "proposed", "rh50": "proposed with stock RH-50 (5 µH)"}
    for d in R["dumps"]:
        ok = (not d["aborted"]) and d["v_drive_max"] <= el.V_DRIVE_MAX - 0.5 and d["v_drain_max"] <= 48 and d["rise_fall"][1] > el.V_FULL + 0.2
        wn = d["worst_node"]
        case = f"{vname[d['variant']]}, {'envelope' if d['envelope'] else 'decel profile'}{' ×%.2f' % d['scale'] if d['scale'] != 1 else ''}, {d['corner']} corner"
        if d["aborted"]:
            L.append(f"| {d['which']} | {case} | {d['regen_P']:.0f} W | {d['rise_fall'][0]:.2f} / {d['rise_fall'][1]:.2f} V | **simulation aborted at "
                     f"{d['t_run'] * 1e3:.2f} ms**: the gate toggles at {d['f_sw_mean'] / 1e6:.0f} MHz (comparator ringing on the clamp's own ESR step) | – | "
                     f"{d['v_drain_max']:.1f} V | {d['i_leg_max']:.0f} A | – | {d['f_sw_mean'] / 1e6:.0f} MHz | ❌ |")
            continue
        L.append(f"| {d['which']} | {case} | {d['regen_P']:.0f} W / {d['regen_E']:.0f} J ({d['t_run'] * 1e3:.0f} ms) | {d['rise_fall'][0]:.2f} / {d['rise_fall'][1]:.2f} V | "
                 f"**{d['v_drive_max']:.2f} V** ({wn['branch']} {wn['node']}) | {_f(d['v_drive_min'], 2, ' V')} | {d['v_drain_max']:.1f} V | {d['i_leg_max']:.0f} A | "
                 f"{d['e_res_each']:.0f} J | {d['f_sw_mean'] / 1e3:.1f} kHz | {'✅' if ok else '❌'} |")
    L += ["", "Nodes: T = shoulder pitch drive (end of the bench lead / lift umbilical), R = roll (after the pitch spiral and the detach plug), Y = yaw, "
          "E = elbow drive + node, W = DM-J4310. Limits: drive ≤ 28 V − 0.5 V = 27.5 V (GIM6010 12–28 V); MOSFET drain ≤ 48 V (80 % of 60 V); the lowest drive "
          "voltage is shown for the ESR dips; the off threshold at the low corner must clear a full pack (> 25.4 V); NH-50 ≤ "
          f"{el.NH50['overload'] * el.NH50['p_rated'] * el.NH50['t_overload'] * el.CLAMP['adiabatic']:.0f} J per pulse (25 % of the 5 s overload rating).", "",
          "**What it shows.** (1) **As drawn, the clamp rings at MHz:** the clamp's turn-on pulls the board node down by ≈ 146 A × C1's ESR plus the film "
          "capacitor's sag (≈ 2 V), more than the 1.2 V hysteresis window, so the 55 ns TLV3202 switches straight off, the node jumps back, and it "
          "switches on again at the gate-edge rate (the simulation aborts on it; on the bench: MOSFETs in linear mode, overheating). The old 36 µs "
          "TLV3702 hid this. Fix: a 2 µs RC on the divider and a 50 µs minimum on-time (one SN74LVC1G123 on OV ↑, ORed into the gate): the chopper "
          "then runs at a few kHz. (2) **The board's threshold is not the drive's voltage:** regen current flows from each drive to the board through "
          "≈ 0.1 Ω of fuse, connectors and 14/16 AWG, so the elbow sits 1–3 V above the board. With v3's window (on at 27.0 V ± 1 %) the drives "
          "exceed 27.5 V already at ≈ 330 W; on at **26.0 V** (0.3 V hysteresis; ± 1 % still clears a full pack, lowest drive ≥ 26 V at the low corner) "
          "buys ≈ 1 V. (3) Even then the bus can take only the regen power below, so the full simultaneous peak-torque stop (1.3 kW per arm) "
          "overshoots: controlled stops must stay under it (drive limits at rated, peak-limit tests one joint at a time, a planner decel limit), "
          "or the branch needs a lower resistance / a local clamp.", "",
          "| Bus | Regen power with every drive ≤ 27.5 V (proposed board, high corner) | Worst-case need (all joints at peak) | Rated-limit stop (×0.45) |", "|---|---|---|---|"]
    for w, rl in R["regen_limit"].items():
        need = next(d["regen_P"] for d in R["dumps"] if d["which"] == w and d["variant"] == "fix" and d["scale"] == 1.0 and d["envelope"] and d["corner"] == "high")
        L.append(f"| {w} | **{rl['P_max']:.0f} W** | {need:.0f} W | {0.45 * need:.0f} W {'✅' if 0.45 * need <= rl['P_max'] else '❌'} |")
    rh = {d["scale"]: d for d in R["dumps"] if d["variant"] == "rh50"}
    nh = {d["scale"]: d for d in R["dumps"] if d["which"] == "M1" and d["variant"] == "fix" and d["envelope"] and d["corner"] == "high"}
    if rh:
        L += ["", "**Stock RH-50 instead of the NH-50 (Fable #2 minor: NH-50 lead time).** Same board with ≈ 5 µH per wirewound resistor (ESTIMATE), freewheel "
              "Schottkys kept, M1 envelope, high corner: " + "; ".join(
                  f"×{sc:.2f}: highest drive {d['v_drive_max']:.2f} V (NH-50 {nh[sc]['v_drive_max']:.2f} V), drain {d['v_drain_max']:.1f} V (NH-50 {nh[sc]['v_drain_max']:.1f} V), "
                  f"chopper {d['f_sw_mean'] / 1e3:.1f} kHz" for sc, d in sorted(rh.items()) if sc in nh) +
              ". With the freewheel path the resistor's L/R (≈ 9 µs) is far under the 50 µs minimum on-time and the drain overshoot is set by the board loop, "
              "not the resistor" + (", so **an RH-50 0.56 Ω is an accepted alternate** wherever the NH-50 lead time exceeds the build plan (the rated-limit stop stays "
              "≤ 27.5 V at the drive, the M1 regen limit moves by a few W); the reviewer re-runs this case on the part actually bought (its inductance is not published)."
              if all(d["v_drain_max"] <= 48 for d in rh.values()) and rh.get(0.45, {}).get("v_drive_max", 99) <= el.V_DRIVE_MAX - 0.5 else
              ", but the rated-limit stop no longer stays ≤ 27.5 V at the drive: keep the NH-50.")]
    L += ["", "## 2. Pre-charge", "", "| Case | PRE_OK after | K1 closes | ΔV at closing → making current | Inrush (K2 + R3) | Energy in R3 | |", "|---|---|---|---|---|---|---|"]
    for p in R["precharge"]:
        if p.get("timeout"):
            what = f"with a {p['dummy']:.0f} Ω dummy load" if p.get("dummy") else f"idle draw ×{p.get('idle_scale', 1):.1f} ({15 * el.I_IDLE_DRIVER * p.get('idle_scale', 1):.2f} A)"
            L.append(f"| {p['which']} {what} | never (bus {p['v_final']:.1f} V, {p['shortfall']:.1f} V short) | PRE_TIMEOUT at 1 s, K2 opens | – | "
                     f"{p['i_inrush']:.1f} A | {p['e_r3']:.0f} J (rating {el.RH50['overload'] * el.RH50['p_rated'] * el.RH50['t_overload']:.0f}) | ✅ |")
        else:
            L.append(f"| {p['which']}, K1 operate {p['t_operate'] * 1e3:.0f} ms, idle ×{p['idle_scale']:.1f} | {p['t_pre_ok'] * 1e3:.0f} ms | {p['t_k1'] * 1e3:.0f} ms | "
                     f"{p['dv_at_close']:.2f} V → {p['i_make']:.0f} A | {p['i_inrush']:.1f} A | {p['e_r3']:.1f} J | {'✅' if p['i_make'] <= 100 else '⚠️'} |")
    i_lim = PRE_OK_DV / el.R_PRE
    L += ["", f"PRE_OK needs the bus's total idle draw under {i_lim:.2f} A (3 V across 3.3 Ω, before the ±0.25 V of comparator tolerance): the robot's "
          f"15 drivers at the {el.I_IDLE_DRIVER * 1e3:.0f} mA ESTIMATE draw {15 * el.I_IDLE_DRIVER:.2f} A, plus the 12 V DC-DC and the nodes downstream of the "
          "contactor; at ×1.5 (0.90 A) the robot never reaches PRE_OK. **Board test T3 measures the real idle draw; the robot needs R3 ≤ 2.2 Ω (with a relay rated for "
          "the 11.5 A inrush) if its idle draw is over ≈ 0.8 A.** K1's making current (ΔV left at closing over ≈ 20 mΩ) is 60–100 A on M1 and 136–152 A on the robot: "
          "check the ALEV100 make rating, or close K1 on a tighter PRE_OK once the idle draw is known.", "",
          "## 3. Watchdog loss → contactor drop → forced discharge", "",
          "| Coil suppression | Q4 | Hold current | Coil current to drop-out (15 %) | Q4 drain peak | |", "|---|---|---|---|---|---|"]
    for c in R["coil"]:
        vds = 30.0 if "30 V" in c.get("q4", "") else 100.0
        L.append(f"| {c['suppression']} | {c.get('q4', '')} | {c['i_hold'] * 1e3:.0f} mA | {_f(c['t_dropout'] and c['t_dropout'] * 1e3, 1, ' ms')} | **{c['v_drain_max']:.1f} V** | "
                 f"{'❌ avalanches at every stop (the Zener path never conducts; V_DS 30 V)' if c['v_drain_max'] > 0.8 * vds else '✅ ≤ 80 % of V_DS'} |")
    L += ["", "| Bus | K1 open → board < UVLO | → far drive < UVLO (node) | Peak discharge current | |", "|---|---|---|---|---|"]
    for d in R["discharge"]:
        L.append(f"| {d['which']} | {_f(d['t_uvlo_board'] and d['t_uvlo_board'] * 1e3, 1, ' ms')} | {_f(d['t_uvlo_far'] and d['t_uvlo_far'] * 1e3, 1, ' ms')} "
                 f"({d['far']}) | {d['i_peak']:.0f} A | ✅ |")
    for w, s in R["stop_chain"].items():
        L += ["", f"**Heartbeat lost → no torque, {w}, worst tolerances:**", "", "| Step | ms |", "|---|---|"]
        L += [f"| {a} | {b * 1e3:.1f} |" for a, b in s["rows"]]
        L += [f"| **total (coil model)** | **{s['total'] * 1e3:.1f}** |", f"| total with the datasheet release ({el.CONTACTOR['t_release'] * 1e3:.0f} ms) instead | {s['total_datasheet'] * 1e3:.1f} |",
              f"| SW2 budget | {s['budget'] * 1e3:.0f} |"]
    L += ["", "The watchdog tolerance is the largest term: at +20 % the 25 ms monostable alone is 30 ms. That is why the review says *K1 open ≤ 30 ms* "
          "does not follow from the part value: it is 32 ms from the last edge to WD_OK low, before the contactor moves. The board test measures "
          "the monostable (T5) and the release (T4) separately and the bench test the end-to-end time, independently of any software timestamp.", "",
          "## 4. Fuse coordination and wire protection", "",
          f"The bench supply latches off at {el.BENCH_PSU['i_max'] * 1.05:.1f}–{el.BENCH_PSU['i_max'] * 1.25:.1f} A; F1 (60 A) never acts on the bench, so the main fuse "
          "is not credited with anything there. Between the branch fuse's rating and the supply trip only the branch fuse protects the wire; its "
          "maximum opening time (Littelfuse ATO: 135 % ≤ 600 s, 200 % ≤ 5 s, 350 % ≤ 0.5 s; 110 % may never open) against the wire's first-order "
          "heating (hot R(T); ESTIMATE G, C per installation):", ""]
    for f in R["fuses"]:
        L += [f"**{f['case']}** — wire: {f['wire']} (continuous ≤ {_f(f['i_continuous_service'], 1, ' A')} in service, ≤ {_f(f['i_continuous_fault'], 1, ' A')} "
              f"to its fault limit {WIRES[f['wire']]['t_max']} °C): {'✅ protected at every current' if f['protected'] else '❌ unprotected at ' + ', '.join(f'{x:.0f}' for x in f['gap']) + ' A'}", "",
              "| Fault current | Opens first | Max time | Wire then |", "|---|---|---|---|"]
        L += [f"| {r['I']:.1f} A | {r['opens']} | {'never' if math.isinf(r['t_open_s']) else _f(r['t_open_s'], 2, ' s')} | {r['T_wire']:.0f} °C {'✅' if r['ok'] else '❌'} |" for r in f["rows"]]
        L.append("")
    L += logic_md(R)
    L += ["", "| Scenario | Netlist | Result | |", "|---|---|---|---|"]
    L += [f"| {r['case']} | {r['variant']} | {r['detail']} | {'✅' if r['ok'] else '❌'} |" for r in R["logic"]]
    L += base_md(R["base"])
    L += ["", "## Design changes from this simulation", ""] + DESIGN_CHANGES + [""]
    return "\n".join(L)


def logic_md(R):
    """§5 head: the Fable #2 M2 race, its fix (netlist v4), the AUX debounce and the before / after numbers."""
    rows = {(r["variant"], r["case"]): r for r in R["logic"]}
    unplug = "K1 coil unplugged while running (T7, T9 b), replugged after 0.25 s"
    db, D, B = R["debounce"], el.AUX_DEBOUNCE, el.K1_BOUNCE
    ok = lambda b: "✅" if b else "❌"
    L = ["## 5. Logic single faults (python model of the netlist)", "",
         "**Fable review #2, M2: the race on an uncommanded K1 drop-out.** In v3 (and with the W13b fixes) K2_DRIVE = LATCH ∧ ¬AUX and U6's CLR̄ = ¬K2_DRIVE. When K1 "
         "opens without the logic asking (coil wire off, coil dip, shock, AUX wire break) AUX falls with LATCH still high: the same edge triggers U6 (Ā ↓) and, two "
         "gates later, raises K2_DRIVE and pulls CLR̄ low. FORCE is a few-ns glitch into U10's CLR̄ that may or may not clear LATCH. The model resolves it as "
         "*no FORCE* (logged as a race): K2 re-closes and pre-charges with LATCH high, and FORCE comes only from the 30 ms discrepancy monitor, while T7 asks for "
         "the far end < 10 V within 4 ms of AUX. The same race fires on every bounce gap at K1 closing, since AUX had no debounce.", "",
         "**Netlist v4 (electrical.md):** (1) **RUN latch U28** (SN74LVC1G74, CLK = AUX_DB ↑, CLR̄ = LATCH): set once K1 has closed in this arm cycle; "
         "**K2_DRIVE = LATCH ∧ ¬RUN ∧ ¬PRE_TIMEOUT**. After K1 has closed, nothing but a new ARM can drive K2 again, so an AUX edge can no longer move U6's CLR̄: "
         "the falling edge fires FORCE cleanly and FORCE clears LATCH (priority to the edge, no race). (2) **AUX debounce U29** (asymmetric RC into a 74LVC1G17): AUX_DB "
         "rises only after the contact has stayed closed for t_stable, and falls within t_fall of a real opening; every AUX input of the logic (U6, U8, U10, U11, U17, U28) "
         "takes AUX_DB, the Pi keeps AUX_RAW. (3) **Discrepancy window 30 → 40 ms** (it must outlast operate + bounce + t_stable). (4) PRE_TIMEOUT reads ¬RUN.", "",
         "| Uncommanded drop (coil unplugged, T7) | FORCE after AUX opens | Far end < 10 V after AUX (T7 ≤ 4 ms) | K2 re-closes (pre-charge with LATCH high) | K1 re-closes when the coil is back |",
         "|---|---|---|---|---|"]
    for var in ("v3", "fix", "v4"):
        r = rows.get((VARIANT_TAG[var], unplug))
        if not r:
            continue
        d = r["detail"]
        L.append(f"| {VARIANT_TAG[var]} | {'none' if r['t7'] is None else '%.1f ms' % (1e3 * (r['t7'] - (R['discharge'][0]['t_uvlo_far'] - T_AUX_LAG)))} | "
                 f"{'never' if r['t7'] is None else '%.1f ms' % (1e3 * r['t7'])} {ok(r['t7'] is not None and r['t7'] <= T7_LIMIT)} | "
                 f"{'**yes**' if r['k2_reclose'] else 'no'} | {'**yes, without ARM**' if 'K1 re-closes' in d else 'no'} |")
    L += ["", f"**AUX debounce, sized from K1's bounce.** {B['src']}: a burst ≤ {B['burst'] * 1e3:.0f} ms after first contact, longest closed interval ≤ "
          f"{B['closed_max'] * 1e3:.0f} ms. Filter: {D['parts']}; τ_rise = R_c·C = {db['tau_c'] * 1e3:.1f} ms, τ_fall = (R_d ∥ R_c)·C = {db['tau_d'] * 1e3:.2f} ms, ± {D['tol']:.0%}; "
          f"thresholds VT+ {D['vt_pos'][0]}–{D['vt_pos'][1]} V, VT− {D['vt_neg'][0]}–{D['vt_neg'][1]} V, ΔVT ≥ {D['hyst_min']} V (ESTIMATE). The logic model runs the "
          "filter as an exact RC per 20 µs step at self-consistent corners and puts the bounce on every v4 closing.", "",
          "| Debounce check | Value | Limit | |", "|---|---|---|---|",
          f"| AUX_DB rises after the contact has stayed closed (t_stable, fast … slow corner) | {db['t_stable'][0] * 1e3:.2f} … {db['t_stable'][1] * 1e3:.2f} ms | – | |",
          f"| opening → AUX_DB low (t_fall, fast … slow corner) | {db['t_fall'][0] * 1e3:.2f} … {db['t_fall'][1] * 1e3:.2f} ms | – | |",
          f"| open glitch ignored while running (fast-fall corner) | < {db['gap_reject'] * 1e3:.2f} ms | – | |"]
    L += [f"| {a} | {v * 1e3:.2f} ms | {'≥' if v >= lim else '≤'} {lim * 1e3:.2f} ms | {ok(g)} |" for a, v, lim, g in db["checks"]]
    L += ["", f"**Discrepancy monitor with the debounce:** XOR(Q4 gate, AUX_DB) now has to outlast K1's operate time ({el.CONTACTOR['t_operate'] * 1e3:.0f} ms max) + the bounce "
          f"+ the slowest t_stable = {db['need'] * 1e3:.1f} ms, so the window goes from 30 ms to **{el.DISC_WINDOW['t'] * 1e3:.0f} ms ± {el.DISC_WINDOW['tol']:.0%}** "
          f"({db['window'][0] * 1e3:.0f}–{db['window'][1] * 1e3:.0f} ms). Its limit: a welded K1 must be caught within one SW2 budget ({el.SW2_BUDGET * 1e3:.0f} ms) of the coil going off: "
          f"{db['window'][1] * 1e3:.0f} ms ✅. It no longer carries the uncommanded drop-out (the AUX_DB edge does, in < {db['t_fall'][1] * 1e3:.1f} ms), so the longer window costs "
          "only the welded-K1 case 10 ms (FORCE at 40 instead of 30 ms after the coil goes off; the drives' own CAN timeout and the supervisor's DISABLE frames are earlier "
          "anyway). The stop chain gains the slow-corner t_fall (§3).", "",
          "**Earlier results (W13b logic, W15b base channel) on v4:** every row that passed still passes (table: the v4 rows and the base-channel rows, which now run "
          "on v4). Two numbers move and are explained: the AUX-wire-break stop now fires FORCE at the AUX_DB edge (< 1 ms) instead of the 30 ms discrepancy, and "
          "a welded K1 is caught at 40 instead of 30 ms after the coil goes off (window above). New rows: the uncommanded drop-outs, the bounce at ARM at both debounce "
          "corners, a 0.1 ms aux glitch while running, and the pre-charge timeout, where the SN74LVC1G123's CLR̄ ↑ trigger fires FORCE once as K2_DRIVE falls "
          "(it empties the partly charged bus; T4 / T7 now expect it)."]
    return L


def base_md(B):
    """§6 of the report: the base channel, the e-stop sequence, the base bus regen and the caster lock at speed."""
    ss1, qs, sn, ps = B["ss1"], B["qs"], B["snag"], B["phase_short"]
    t_min = ss1["t_set"] * (1 - ss1["tol"])
    ok = lambda b: "✅" if b else "❌"
    rc = next((r for r in B["regen"] if r["clamp"]), {})
    nc = [r for r in B["regen"] if not r["sink"] and not r["clamp"]]
    L = ["", "## 6. Base channel: the category-1 base stop (W14a docs/sw/base_stop.md)", "",
         "**Netlist (robot board; electrical.md *Safety board*, refs U20–U26, Q9–Q10).** The base branch no longer hangs after K1. "
         f"**Q9 base switch** ({el.BASE_SWITCH['name']}: its soft-start into the drives' 1.9 mF is the base pre-charge, {el.BASE_SWITCH['i_limit']:.0f} A circuit breaker) "
         f"feeds VBUS_B → F 20 A → the 4 wheel cartridges, and the caster-lock release ({el.CASTER_SOL['part']}, ≈ {el.CASTER_SOLENOID_A:.1f} A while released, ESTIMATE). "
         "Its enable runs through the two OFF-delay safety contacts of "
         f"**U20, {ss1['name']}**; U20's dual-channel input is the mushrooms' third and fourth NC contact blocks (S1c/S2c, S1d/S2d) with a photoMOS "
         "driven by LATCH in channel 1. So **any e-stop or board STOP starts a fixed delay and the base keeps power during it**, and after "
         f"{ss1['t_set']:.1f} s U20 removes the base enable whatever the software does; only ARM (SB1's second contact on U20's reset) re-energises it. "
         "**WD_B** (a second SN74LVC1G123, 25 ms, GPIO23 toggled by `base_heartbeat_ok()`) is a board STOP term and an ARM condition: a dead base loop "
         "stops the arm (category 0) and starts the base delay. **Base clamp leg Q10** on VBUS_B (one more leg of the same parts, same window) with "
         "**FORCE_B**, a 100 ms one-shot on the enable's falling edge, and **DISC_B**, the base bus still up 50 ms after the enable dropped (a shorted Q9) → "
         "GPIO + the BMS discharge FET off over CAN, the robot's second break.", "",
         "### E-stop sequence (board event model at 0.1 ms + the base's speed; arm watchdog at +20 %)", "",
         f"Transit: {B['M']:.1f} kg (twin), 0.8 m/s with the casters released, ramp at {B['a_transit']:.1f} m/s² (CASTER_RELEASED_A, the limit in force); "
         f"basket: +9.07 kg at {B['v_basket']:.2f} m/s (the locked cap CASTER_LOCKED_V, lowered from 0.3 in W16a), ramp at {B['a_basket']:.1f} m/s² (load mode). "
         "Arm torque off = K1 open + far drive below UVLO (§3) + 1 ms. "
         f"Base power off = U20's delayed contacts open (Q9 off; FORCE_B then empties VBUS_B in {_f(rc.get('t_uvlo_after_off') and 1e3 * rc['t_uvlo_after_off'], 1, ' ms')}). "
         f"Caster lock = SW-T2 as implemented in W16a (sw/safety.py `_caster_tick`): a forced lock is deferred until the base is at ≤ V_CASTER_LOCK = {V_CASTER_LOCK} m/s, "
         "or its power is off (the solenoid drops with VBUS_B). The W15b column *forced at the fault latch* is gone: that was the software before W16a.", "",
         "| Case | Arm torque off | Base stopped | Base power off | Speed at power off | Distance | Casters lock (s @ m/s) | |",
         "|---|---|---|---|---|---|---|---|"]
    t_uv = 2.8e-3 + 1e-3
    for q in B["seqs"]:
        arm = "— (no power)" if q["power_lost"] else f"{(q['t_k1_open'] + t_uv) * 1e3:.0f} ms"
        stop = f"{q['t_stopped']:.2f} s" if q["t_stopped"] is not None else "coasts out"
        good = (q["t_stopped"] is not None and q["t_stopped"] <= q["t_base_off"]) or q["v_at_base_off"] <= V_CASTER_LOCK + 0.011
        lp = q["lock_sw"]
        off = (f"0 s (all power)", f"{q['v0']:.2f} m/s") if q["power_lost"] else (f"{q['t_base_off']:.2f} s", f"{q['v_at_base_off']:.2f} m/s")
        L.append(f"| {q['name']} | {arm} | {stop} | {off[0]} | {off[1]} | {q['x_stop']:.2f} m | "
                 f"{lp[0]:.2f} @ {lp[1]:.2f} | {'⚠️ residual' if q['power_lost'] else ok(good)} |")
    L += ["", f"Pass: the base is stopped before U20 removes its power, or at most ≈ {V_CASTER_LOCK} m/s is left (then it coasts out with the casters locked). "
          f"U20 at its low corner ({t_min:.1f} s) still clears the longest software ramp ({B['ramp_max']:.1f} s; with the {B['grace']:.1f} s grace "
          f"{B['ramp_max'] + B['grace']:.1f} s, which only matters if the wheels never report a stop). With the Pi hung the drives' own CAN-timeout "
          f"quick-stop does the ramp: at {qs['a_max']:.1f} m/s² it stops in time; at the {qs['a_default']:.1f} m/s² default it leaves ≤ 0.26 m/s at the "
          "timer, which coasts out. A pack / supply loss is the residual case: the base coasts from full speed with the casters spring-locked "
          "(no stop function survives losing all power; the planner's speed is the only bound).", "",
          "### Regen on the base bus during the ramp (ngspice)", "",
          "With K1 open the arm's clamp and FORCE are on the other bus, so the base's braking energy goes to VIN. On the robot VIN is the pack, which "
          "absorbs it, unless the BMS has its charge FET off (pack full after docking / over-voltage, charging below 0 °C, a BMS fault): then only "
          "the always-on logic load and the drives' idle draw take it.", "",
          "| Pack | Logic load | Drive idle | Base clamp leg | Regen peak / energy | Highest drive voltage | Clamp energy (ramp) | Q9 off → drives < UVLO | |",
          "|---|---|---|---|---|---|---|---|---|"]
    for r in B["regen"]:
        L.append(f"| {'sinks (charge FET on)' if r['sink'] else 'cannot sink (charge FET off)'} | {r['logic_w']:.0f} W | ×{r['idle_scale']:.1f} | {'yes' if r['clamp'] else 'no'} | "
                 f"{r['P_peak']:.1f} W / {r['E_regen']:.1f} J | **{r['v_drive_max']:.2f} V** | {_f(r.get('e_leg_ramp'), 1, ' J') if r['clamp'] else '–'} | "
                 f"{_f(r.get('t_uvlo_after_off') and r['t_uvlo_after_off'] * 1e3, 1, ' ms') if r['clamp'] else 'idle draw only (> 30 ms)'} | {ok(r['v_drive_max'] <= el.V_DRIVE_MAX - 0.5)} |")
    L += ["", f"**Reading:** the ramp returns only ≈ {B['regen'][0]['E_regen']:.0f} J at ≤ {B['regen'][0]['P_peak']:.0f} W, but with the pack unable to sink it the bus "
          f"rises unless something on VIN burns it: the logic load ({el.LOGIC_MIN_W:.0f} W ESTIMATE) and the drives' idle draw just cover it at the estimates, and at "
          f"half of them the drives see {min(r['v_drive_max'] for r in nc[1:]):.0f}–{max(r['v_drive_max'] for r in nc):.0f} V. **So the base bus gets its own clamp leg** "
          f"(one more NH-50 leg, same window, its own FORCE_B), which holds {rc.get('v_drive_max', float('nan')):.1f} V with no logic load at all and empties VBUS_B "
          "within ms when Q9 opens. It must not be the arm clamp through a diode: FORCE on K1's opening would then empty the base bus too (base_stop.md item 5).", "",
          "### No phase-short braking (requirement 4)", "",
          f"A wheel drive that shorts its phases on disable or power loss brakes each GIM6010-8 with its back-EMF into its own winding (output Ke ≈ Kt "
          f"{el.MOTOR_CU['GIM6010-8'][0]:.2f} N·m/A, 2 × {el.MOTOR_CU['GIM6010-8'][1]} Ω line to line, ESTIMATE): at 0.8 m/s ≈ {ps['i']:.0f} A per motor, "
          f"**≈ {ps['a_lin']:.1f} m/s²** on the unloaded robot, falling ∝ speed. Tip table (tip.md): transit tips at 3.5 m/s², the R2d basket at 1.0 m/s² "
          f"(a basket carry at 0.3 m/s would see ≈ {ps['a_lin'] * 0.3 / 0.8 * B['M'] / (B['M'] + 9.07):.1f} m/s²). It stays under 0.5 m/s² only below "
          f"{ps['v_ok']:.2f} m/s. **Drive configuration requirement:** on disable, after the CAN-timeout quick-stop and at UVLO the drive floats its "
          "phases (coast); no brake-on-disable / short-brake mode. Verify at bench bring-up with the GIM's CAN timeout (sw.gim_discover).", "",
          "### Caster lock at speed (W14a open point; D18b, tip.md)", "",
          "**Engaging the lock itself is harmless at any speed:** the spring tips the plate onto the Ø8 rod, which on a flat floor is not moving "
          "vertically, so there is no shock in the lock and no braking force (a locked foot carries the same 5 N preload as a released one, ≈ 1 N of "
          "PTFE drag). **Travelling locked at speed is not harmless:** locked, the foot can follow the floor down but not be pushed up, so any "
          "step edge in its path (rug edge, tile lip, a cable, the far side of a dip it ratcheted into) is a rigid floor-level snag. At speed the base "
          "cannot be lifted over the 44° flank in time, so the model is a dead stop at the leading caster(s), with angular momentum about the snag line kept and the "
          f"impulse taken over {sn['delta'] * 1e3:.0f} mm of contact compliance (ESTIMATE). Limits: snag energy ≤ {sn['frac_lim']:.0%} of the tip-over energy; force per "
          f"foot ≤ ½ × {sn['F_rod']:.0f} N (two Ø8 rods yielding at a {ROD['lever'] * 1e3:.0f} mm lever, ESTIMATE; the printed guide is not rated: bench).", "",
          "| State | Speed | Worst heading (casters on the snag line) | COM behind the line | Energy / tip energy | Pitch reached | Max force per foot (any heading) | |",
          "|---|---|---|---|---|---|---|---|"]
    for r in sn["rows"]:
        good = r["frac"] <= sn["frac_lim"] and r["F_max"] <= 0.5 * sn["F_rod"]
        L.append(f"| {r['state']} ({r['M']:.1f} kg, COM h {r['h']:.2f} m) | {r['v']:.2f} m/s | {r['az']}° ({r['n_snag']}) | {r['x'] * 1e3:.0f} mm | "
                 f"{r['frac']:.0%}{' **tips**' if r['frac'] >= 1 else ''} | {r['theta_deg']:.1f}° | {r['F_max']:.0f} N | {ok(good)} |")
    fast = [r for r in sn["rows"] if abs(r["v"] - 0.8) < 1e-9]
    L += ["", f"**Verdict: the casters must not lock at 0.8 m/s.** A snag at 0.8 m/s uses {min(r['frac'] for r in fast):.0%}–{max(r['frac'] for r in fast):.0%} of the "
          f"tip energy (it tips with the lift at the top; the tip speed is {sn['v_tip']:.2f} m/s) and puts ≈ {max(r['F_max'] for r in fast) / 1e3:.1f} kN on a foot whose "
          f"rods yield at ≈ {sn['F_rod']:.0f} N. The lowest allowed speed over the states and headings is {sn['v_allow']:.2f} m/s (energy {sn['v_frac']:.2f}, rod force "
          f"{sn['v_force']:.2f} m/s), so **the forced lock waits for ≤ {V_CASTER_LOCK} m/s** {ok(sn['lock_ok'])} (sw/safety.py V_CASTER_LOCK, W16a). The locked-travel cap "
          f"(CASTER_LOCKED_V {sn['v_locked_cap']} m/s, W16a; it was 0.3 m/s, just over the rod-force limit on the diagonal heading) is "
          f"{'within' if sn['locked_cap_ok'] else 'over'} the limits in this model {ok(sn['locked_cap_ok'])}. Open (W16a log, not this board's): the basket at 0.25 m/s in the "
          "tuck-pose model; a bench snag test of a locked foot into a 10 mm step is still the evidence to get.", "",
          "**SW-T2 status (sw/safety.py `_caster_tick`, implemented in W16a; this list is now the check of it):**", "",
          f"1. A forced lock (a latched fault, a person approaching or near, a payload past the mode's limit) does **not** de-energise the "
          f"solenoid while the base moves faster than **V_CASTER_LOCK = {V_CASTER_LOCK} m/s**. The casters stay released, the base ramps at the "
          f"released limit (CASTER_RELEASED_A {B['a_transit']:.1f} m/s², or the fault ramp's own rate if lower), and the lock engages when the measured "
          f"speed (odometry, `limit_base(v_meas=…)`; without odometry the ramp's command) is ≤ {V_CASTER_LOCK} m/s: ≈ "
          f"{(0.8 - V_CASTER_LOCK) / B['a_transit']:.1f} s and {(0.8 ** 2 - V_CASTER_LOCK ** 2) / (2 * B['a_transit']):.2f} m into a 0.8 m/s ramp (the table's lock column).",
          "2. A threshold release (≤ 0.12 m/s) is already below the limit, so it locks at once.",
          f"3. Hardware still locks without software. The solenoid hangs on VBUS_B, so it drops when U20 opens Q9 (≤ {ss1['t_set'] * (1 + ss1['tol']):.1f} s; "
          "by then even the drives' default quick-stop has the base ≤ 0.26 m/s), and at any power loss (spring-applied). The solenoid driver's gate "
          "comes from the Pi through a pull-down, so a dead Pi cannot hold the casters released past U20. With one solenoid pulling four cables (D29 #9) a stuck "
          "plunger or a kinked liner holds all four released (tip.md): the bench pull-force test covers it.",
          "4. Counted (\"caster lock deferred\"); `casters_locked()` stays False until it engages, so the planner and tip.py see the real state.",
          "5. Residual: a total power loss in transit spring-locks the casters at up to 0.8 m/s. Transit runs only unloaded with the arms parked and "
          "nobody near. Park the lift low for transit: a snag with the lift mid uses less of the tip energy than with it at the top (table).", ""]
    return L


DESIGN_CHANGES = [
    "0. **Clamp comparator: 2 µs RC on the divider + a 50 µs minimum on-time** (one more SN74LVC1G123 on OV ↑, ORed with OV and FORCE before the "
    "UCC27524). As drawn the clamp's own turn-on step at the board node is larger than its hysteresis and it rings at MHz (section 1). "
    "**Thresholds: on at 26.0 V, off at 25.7 V** (was 27.0 / 25.8 V as the corners were computed): ± 1 % still clears a full pack and every drive gains ≈ 1 V.",
    "1. **Q4 (K1 coil driver) → 100 V logic-level MOSFET** (e.g. an IRL540N-class part, V_GS(th) ≤ 2 V, R_DS(on) specified at 4–5 V; drive it from "
    "the UCC27524's spare channel at 12 V if no 100 V part is specified at 3.3 V). With D1 + Z1 across the coil the drain sees supply + V_Z + V_F ≈ 50 V at "
    "every turn-off; the IRLB8721 is a 30 V part. Q5 (K2) likewise ≥ 60 V.",
    "2. **ARM needs AUX open:** U10's D input becomes S1b ∧ S2b ∧ WD_OK ∧ TH1 ∧ ¬AUX. A stuck-closed AUX (welded aux, shorted opto) otherwise lets K1 "
    "close straight onto an empty bus (Q4 holds on AUX, K2_DRIVE = ¬AUX never pre-charges) and removes the opening edge that fires FORCE.",
    "3. **K1 discrepancy monitor:** coil command (Q4 gate) ≠ AUX for > 30 ms (longer than the 25 ms operate time; **40 ms in v4**, design change 10) fires FORCE and clears LATCH (an XOR, "
    "an RC and a comparator into U6's trigger and U10's CLR̄). It catches a welded K1 (coil off, AUX closed: the clamp loads the bench supply at ≈ 135 A, "
    "which latches it off, so power is removed without software), a broken AUX wire (coil on, AUX open: as drawn K2 re-closes, U6 is held cleared and "
    "the next stop has no forced discharge) and a K1 that fails to close. On the robot the pack keeps a welded bus up: the robot also needs a second "
    "break (BMS discharge MOSFET off over CAN / its enable input) on the same stop.",
    "4. **12 V supervision:** a 10.5 V UVLO comparator on the gate-driver supply in series with WD_OK (or U2's power-good into U8): without the 12 V the "
    "clamp and FORCE are dead while the contactor can still close.",
    "5. **Regen at the drive, not just at the board:** the board clamps its own node; each drive sits behind its harness resistance, so regen current "
    "lifts it above the board. The branch limits above are a requirement on the drive torque limits and the planner (≤ that regen power per branch "
    "during a controlled stop), and the bench scopes the bus **at the drive** (T8). If a joint needs more, it needs a thicker branch or a local clamp.",
    "6. **Branch fuse vs the in-arm 16 AWG:** a 20 A slow-blow leaves the clock-spring 16 AWG unprotected from ≈ 18 A to ≈ 35 A (it may take minutes "
    "at 22–30 A; section 4), and a 15 A fuse still leaves ≈ 20 A for up to 600 s (and is under 1.25 × the arm's 14.8 A sustained). The fuse stays 20 A "
    "for shorts; overloads get a **branch current monitor**: a Hall sensor per arm branch on the board (ACS724-30A class) → 5 s RC → comparator at "
    "16 A → STOP (into U10's CLR̄, latched). Needed before the arm's own harness is powered (Step 2); Step 1 (≤ 13.8 A peak on a 14 AWG lead) is "
    "protected by the 20 A fuse and the supply latch as it is.",
    "7. **Base channel off K1 (robot board, §6; W14a base_stop.md items 1–2):** Q9 solid-state base switch (LM5069-1 + IRFB7530; its soft-start is the "
    "base pre-charge) from VIN, enabled through the OFF-delay safety contacts of U20 (Omron G9SA-321-T075, set 2.0 s), whose dual-channel input is a "
    "third and fourth NC block on each mushroom plus LATCH; WD_B (base heartbeat, 25 ms) is a STOP and ARM term; the caster-release solenoid on VBUS_B. "
    "E-stop: arm torque off at 14 ms (mushroom) / 42 ms (hung loop), base ramped, base power off at 2.0 s ± 10 % at the latest. Not on the M1 bench (no base).",
    "8. **Base clamp leg + FORCE_B + DISC_B** (§6 regen): with the BMS charge FET off the base's ramp energy has nowhere to go but the logic load; "
    "at half the load / idle estimates the drives see 36–45 V. One more NH-50 leg on VBUS_B holds 26.3 V; FORCE_B empties VBUS_B when Q9 opens; "
    "DISC_B catches a shorted Q9 (→ BMS discharge FET off, the second break).",
    "9. **Requirements outside the board:** wheel drives float their phases on disable / CAN timeout / UVLO (a phase short brakes ≈ 2.8 m/s² at "
    "0.8 m/s); drive quick-stop on CAN timeout ≤ 0.3 m/s² by default, written to the supervisor's base_stop_decel() (≤ 0.5) when it changes; "
    "SW-T2's forced caster lock waits for ≤ 0.25 m/s (a locked caster at 0.8 m/s turns a floor step into a snag that can tip the robot): **done in W16a** "
    "(sw/safety.py V_CASTER_LOCK, CASTER_LOCKED_V 0.25 m/s).",
    "10. **Netlist v4, Fable #2 M2 (the uncommanded drop-out race) + AUX debounce:** a RUN latch (U28 SN74LVC1G74: CLK = AUX_DB ↑, CLR̄ = LATCH) and "
    "K2_DRIVE = LATCH ∧ ¬RUN ∧ ¬PRE_TIMEOUT, so a K1 opening the logic did not command fires FORCE on its own edge and K2 never re-closes; AUX debounced by an "
    "asymmetric RC + 74LVC1G17 (U29: AUX_DB rises after ≥ 2.1 ms closed, falls ≤ 0.5 ms after an opening), sized from the K1 bounce ESTIMATE (3 ms burst, 1 ms "
    "longest closed interval; T3 measures it); every logic input reads AUX_DB; the discrepancy window 30 → 40 ms ± 10 %; PRE_TIMEOUT on ¬RUN. "
    "Coil unplugged (T7): FORCE 30 ms → < 1 ms after AUX, far end < 10 V at ≈ 2 ms (≤ 4 ms), no pre-charge re-close (section 5).",
]


CLAMP_BW = 800e3            # Hz, Micsig CP2100A (m1_kit.py CLAMP_100A)


def _read_exp(tau, bw=CLAMP_BW):
    """Peak a first-order probe of bandwidth bw reads on an exponential decay of time constant tau (step onset), as a fraction."""
    tc = 1 / (2 * math.pi * bw)
    if tau <= tc * 1.0001:
        return tau / (tau + tc)
    ts = tau * tc * math.log(tau / tc) / (tau - tc)
    return tau / (tau - tc) * (math.exp(-ts / tau) - math.exp(-ts / tc))


def _read_pulse(w, bw=CLAMP_BW):
    """Fraction of a flat-top pulse of width w a first-order probe reaches."""
    return 1 - math.exp(-w / (1 / (2 * math.pi * bw)))


def clamp_bw_rows(R):
    """Each test that uses the 100 A clamp: the fastest current feature from the simulation, and what the CP2100A reads."""
    C = el.c_bus("M1")
    coil = R["coil"][1]
    d045 = next(d for d in R["dumps"] if d["which"] == "M1" and d["variant"] == "fix" and d["scale"] == 0.45 and d["corner"] == "high")
    tau_fw = el.NH50["L"] / el.CLAMP["r_each"]
    tau_dis = el.CLAMP["r_each"] / 3 * C
    rows = [("T3", "pre-charge inrush (supply lead)", f"RC decay τ = {el.R_PRE * C * 1e3:.0f} ms", _read_exp(el.R_PRE * C)),
            ("T6", "K1 coil current to drop-out", f"{coil['t_dropout'] * 1e3:.1f} ms decay", _read_exp(coil["t_dropout"])),
            ("T7", "forced discharge, clamp leg 1 (≈ 41 A)", f"RC decay τ = {tau_dis * 1e3:.1f} ms", _read_exp(tau_dis)),
            ("T11 / T12", "clamp leg 1 on-pulses (≈ 44 A)", f"≥ {T_ON_MIN * 1e6:.0f} µs flat top (min on-time; chopper {d045['f_sw_mean'] / 1e3:.1f} kHz)", _read_pulse(T_ON_MIN)),
            ("T11", "freewheel diode current after turn-off", f"L/R decay τ = {tau_fw * 1e6:.1f} µs (NH-50 1 µH / 0.56 Ω)", _read_exp(tau_fw)),
            ("T16", "supply lead at the SE-1000 trip", "the supply's ms shutdown", _read_exp(1e-3))]
    return [(a, b, c, f"{100 * r:.0f} %", r >= 0.98) for a, b, c, r in rows]


def review_gate_md():
    """Fable #2 M2: the 'someone who did not draw the board' gate had no owner. What it covers, who qualifies, what evidence
    closes it, and what happens if nobody can be found. The owner picks the person."""
    return [
        "## Review gate G-SB: independent schematic + layout review (owner picks the reviewer)", "",
        "**Where it sits:** after the board is drawn in KiCad and `python -m rnd.safety_board_sim` passes on the drawn values; before any Step-1b board part is "
        "ordered (M1_READINESS: Step 1b waits for it) and long before T0. Step 1a (the motor on a 1 A limit, no board) does not wait for it.", "",
        "**What the review covers (all of it; a partial review does not close the gate):**", "",
        "1. Every line of the checklist below (schematic, layout, before sign-off), signed or with a defect written against it.",
        "2. The logic as a state machine, not gate by gate: ARM, pre-charge, run, every stop (commanded, uncommanded K1 drop-out, welded K1, stuck / broken AUX, "
        "12 V loss, POR), each traced on the schematic with the netlist v4 race fix (RUN latch, AUX debounce, discrepancy window); the reviewer writes down any "
        "sequence the simulation's §5 does not cover.",
        "3. The power stage: the clamp loop, the 146 A path, the freewheel diodes, Q4's drain rating, the K1 / K2 / fuse DC ratings, re-derived from the datasheets.",
        "4. The pass / fail numbers of T0–T16 (are they the right tests, are the limits right, can the bench equipment measure them).",
        "5. A short written verdict: build as drawn / build after the listed fixes / do not build.", "",
        "**Who qualifies (any one route; never the drafter, i.e. Claude or any agent in this repo, never the owner alone, never an AI review: Fable, Astra and "
        "Neon are design reviews, not this gate):**", "",
        "| Route | Who | Minimum qualification | Cost / time (ESTIMATE) |", "|---|---|---|---|",
        "| A. Hired EE review | one electrical engineer engaged for a fixed-scope review (freelance platform, a local electronics design consultancy, or a "
        "university power-electronics lab doing paid work) | degree-level EE plus ≥ 5 years designing power electronics, motor drives or machine safety circuits "
        "(a PE / CEng / IEng, or a TÜV functional-safety engineer, counts on its own); has designed a board that switches ≥ 20 A DC | 4–8 h, ≈ $400–1,200; 1–2 weeks |",
        "| B. Maker-community review | **two** independent reviewers from a public electronics community (EEVblog forum, KiCad forum, an open-source robotics / ODrive / "
        "VESC / BMS project's contributors) | each with a public track record: a published board that switches ≥ 20 A DC or a published safety / BMS / motor-controller "
        "design, linkable; both do the whole checklist (a thread of 'looks fine' replies is not a review) | free or a small honorarium; 2–4 weeks |",
        "| C. Bought-relay fallback | route A or B on a smaller board | as A or B | see below |", "",
        "**Minimum evidence to accept it (all five, filed as docs/review/board_review_<date>.md with the attachments):**", "",
        "1. Reviewer identity and qualification (name, route A / B, a link or CV line for the qualification above) and a one-line conflict statement "
        "(no stake in the build passing).",
        "2. The exact files reviewed: the KiCad project's git commit hash, and the electrical.md / safety_board_sim.md revisions.",
        "3. This checklist with every line signed or a defect written (route B: one per reviewer), plus the annotated schematic PDF or the reviewer's ERC / DRC "
        "report, so the review is visibly of the drawing and not of the prose.",
        "4. A defect list with severity (stop / fix before build / note); every 'stop' and 'fix' closed by a commit the reviewer re-checks (a delta review of the "
        "changed sheets), or accepted in writing by the reviewer.",
        "5. The reviewer's own re-run of `python -m rnd.safety_board_sim` on the drawn values (its summary pasted in), or a statement of which numbers they re-derived by hand instead.", "",
        "**If no reviewer is found within 3 weeks of the board being drawn** (the gate must not stall Step 1 indefinitely; Fable #2 M2): route C. Move the e-stop "
        "loop, the monitored reset and the welded-contact check to a bought safety relay with monitored manual reset and contactor feedback (the G9SA family the "
        "base channel already uses), keep the custom board for the clamp, pre-charge and watchdog only, and review that smaller board by route A or B. That removes "
        "about ten single-gate ICs (U6's race path, U10, U11, U17, U28, U29 among them) and most of the logic review. The owner chooses the route and the person; "
        "this page only fixes what the review must cover and what closes it.", ""]


def tests_md(R):
    """Motorless board test procedure with pass/fail numbers (Step-1 gate)."""
    d_m1 = next(d for d in R["dumps"] if d["which"] == "M1" and d["envelope"] and d["scale"] == 1.0 and d["corner"] == "high")
    p_m1 = next(p for p in R["precharge"] if p["which"] == "M1" and not p.get("timeout") and p["t_operate"] == T_K1_OPERATE[1])
    p_dum = next(p for p in R["precharge"] if p.get("timeout") and p["which"] == "M1")
    d045 = next(d for d in R["dumps"] if d["which"] == "M1" and d["variant"] == "fix" and d["scale"] == 0.45 and d["corner"] == "high")
    lim = R["regen_limit"]["M1"]["P_max"]
    dis = R["discharge"][0]
    sc = R["stop_chain"]["M1"]
    wd = el.WATCHDOG["t_nom"]
    tau_pre = el.R_PRE * el.c_bus("M1")
    # T12's 30 A step: the 20 A supply + the 22 mF bank through a resistor, 15 → 10 A extra over ≥ 3 ms (≥ 10 chopper cycles)
    v_lim_tp2 = d045["drives"]["n0T"] + 0.3
    bank = dict(C=0.022, r=0.33, i0=15.0, i1=10.0)
    bank["v0"] = v_lim_tp2 + 0.5 + bank["i0"] * bank["r"]
    bank["t"] = bank["r"] * bank["C"] * math.log(bank["i0"] / bank["i1"])
    bank["cycles"] = bank["t"] * d045["f_sw_mean"]
    clamp_rows = clamp_bw_rows(R)
    B = R.get("base")
    db = R["debounce"]
    t_far = dis["t_uvlo_far"] - T_AUX_LAG
    v4 = {r["case"]: r for r in R["logic"] if r["variant"] == VARIANT_TAG["v4"]}
    uv = v4["K1 coil unplugged while running (T7, T9 b), replugged after 0.25 s"]
    aw = v4["AUX wire breaks while running"]["t_force"]
    L = ["# Safety board: motorless bench tests (generated by `python -m rnd.safety_board_sim`)", "",
         "**Step-1 gate (M1_READINESS exit criterion 1):** every test below passes on the built board, with no motor connected, before any drive is "
         "powered from it. Numbers come from the simulation ([safety_board_sim.md](safety_board_sim.md)) with tolerances; a reading outside the pass band "
         "stops the build until it is understood. Record every scope capture (PNG + CSV) in docs/sw/board_tests/ with the date and the board revision.", "",
         "## Equipment", "",
         f"- Bench supply: the SE-1000-24 set to **24.0 V** (Fable #2 minor: at 25.2 V the clamp's lowest turn-off, {corners('fix')['low'][1]:.2f} V, is only 0.24 V away "
         "against the supply's ± 1 % and 150 mV ripple, and a drift makes the clamp chop against the supply; 24.0 V leaves ≈ 1 V; the simulation's 25.2 V full pack stays "
         "the worst case for the regen numbers) (and the 0–30 V / 10 A CC supply for T0 and T9); a second CC supply (0–30 V) for the clamp tests.",
         "- **Dummy drive load:** a capacitor bank of 2.35 mF (5 × 470 µF 35 V, the M1 drivers' input capacitance) on 0.5 m of 14 AWG at the end of the real "
         "1.5 m arm lead + F3, plus switchable resistors: 10 Ω 50 W (RH-50, pre-charge fault), 5.6 Ω and 2.2 Ω 50 W on the heatsink.",
         "- **Regen source:** the 0–30 V / 20 A CC supply in series with a **40 A Schottky** (Vishay VS-40CPQ060-N3, 60 V, both 20 A legs in parallel, "
         f"on a small heatsink: ≈ 0.5 V × 30 A ≈ 15 W for 200 ms; the 1N5822 named before is a 3 A part) into the dummy drive bank (it pushes current "
         "*into* the bus, which the SE-1000 cannot sink); ≈ 500 W of simulated regen. **The 20 A supply cannot make T12's 30 A step:** the extra "
         f"10 A comes from the energy bank (T12). Energy bank: 22 mF 50 V, charged to 30 V for the dumps (9.9 J … repeat 10× in 1 s for the pulse duty) "
         f"and to {bank['v0']:.1f} V for the 30 A step, switched in through {bank['r']} Ω and a second VS-40CPQ060-N3.",
         "- 4-channel scope (≥ 100 MHz, 1 GS/s) with ×10 passive probes (≥ 300 V) and ground springs: **no differential probe** (Fable #2 minor: Q1–Q3, Q4 and Q5 are "
         "low-side switches, so every drain is measured to board ground, and the SE-1000 output floats), a 100 A DC current clamp (**≥ 500 kHz**: the Micsig CP2100A's 800 kHz "
         "is enough, see *Current-clamp bandwidth* below) and a 30 A clamp; a function generator "
         "(3.3 V square, 250 Hz = the 500 Hz heartbeat toggle) for GPIO17; a thermocouple on the resistor plate; DMM; a stopwatch log (the scope timebase is "
         "the reference, not any software timestamp).",
         "- Scope points: **TP1 VBUS at the board**, **TP2 VBUS at the dummy drive bank** (end of the arm lead), **TP3 Q1 drain** (×10 probe, ground spring on Q1's source), "
         "**TP4 Q1 gate**, TP5 WD_OK, TP6 Q4 gate, **TP7 Q4 drain**, TP8 AUX_RAW (opto output), TP9 FORCE, TP10 LATCH, TP11 K2_DRIVE, **TP12 AUX_DB** (debounced), "
         "**TP13 RUN**; current: clamp leg 1, supply lead, K1 coil.", "",
         "## Tests (pass / fail)", "",
         "| # | Test | Setup and stimulus | Measure | Pass | Fail → |", "|---|---|---|---|---|---|"]
    T = [
        ("T0", "First power, logic only", "CC supply at 24.0 V / 1 A limit, no load, no heartbeat", "supply current; 12 V and 3.3 V rails; LATCH",
         "≤ 0.15 A; 12 V ± 5 %, 3.3 V ± 3 %; LATCH 0, K1 and K2 open", "stop: rails / shorts"),
        ("T1", "Power-on comes up disarmed", "supply off → on 10 times, heartbeat running, ARM not pressed; then 5 times with ARM held down during power-up",
         "LATCH, K1, K2 (TP10, AUX)", "LATCH stays 0 every time (R9·C9 POR ≥ 50 ms on TP10)", "POR network"),
        ("T2", "ARM refused", "(a) a mushroom pressed (each one, each channel); (b) no heartbeat; (c) TH1 open (unplug); (d) **AUX forced high** (short the opto output) with the fix",
         "LATCH after SB1", "LATCH stays 0 in every case", "U10 D gating"),
        ("T3", "Pre-charge into the dummy bank", f"heartbeat on, ARM; dummy bank 2.35 mF + board 4.7 mF; measure the real idle draw first (drives' logic boards if available)",
         "TP1/TP2 rise, K2 / K1 timing, supply current; **AUX bounce**: TP8 + TP12 at 500 µs/div (single shot on TP8 rising) on 20 closings",
         f"PRE_OK at {p_m1['t_pre_ok'] * 1e3:.0f} ms ± 30 % (τ = {tau_pre * 1e3:.0f} ms); inrush ≤ {el.V_FULL / el.R_PRE * 1.1:.1f} A; "
         f"K1 closes ≤ {(p_m1['t_k1']) * 1e3 + 10:.0f} ms after ARM with ΔV ≤ {max(p_m1['dv_at_close'], 0.5):.1f} V; K2 open ≤ {(db['t_stable'][1] + T_K2_RELEASE) * 1e3 + 1:.0f} ms after TP8 first closes "
         f"(t_stable ≤ {db['t_stable'][1] * 1e3:.1f} ms + K2 release); idle draw × 3.3 Ω ≤ 2.0 V; bounce burst ≤ {el.K1_BOUNCE['burst'] * 1e3:.0f} ms with no closed interval "
         f"> {db['closed_reject'] / 1.5 * 1e3:.1f} ms, and one clean TP12 edge per closing (else re-size C_db: design change 10)",
         "pre-charge / PRE_OK divider / debounce"),
        ("T4", "Pre-charge fault", "10 Ω RH-50 across VBUS, ARM", "TP1, LATCH, K2, R3 temperature",
         f"bus settles ≈ {p_dum['v_final']:.1f} V (< VIN − 3 V), K1 never closes, PRE_TIMEOUT clears LATCH at 1.0 s ± 0.2 s, K2 opens; R3 ≤ {p_dum['e_r3'] * 1.2:.0f} J; "
         "FORCE (TP9) fires exactly once, as K2_DRIVE falls (the SN74LVC1G123's CLR̄ ↑ trigger), never before",
         "U11 timer"),
        ("T5", "Watchdog timeout", "armed, running; stop the 250 Hz square at a random phase (20 runs); also 3 runs each at 4 / 8 / 12 ms edge spacing (must not trip)",
         "last edge → WD_OK low (TP5) → Q4 gate (TP6) → AUX (TP8)", f"WD_OK low {wd * 1e3 * 0.9:.1f}–{wd * 1e3 * 1.2:.1f} ms after the last edge (25 ms ± 10 % with a C0G cap; "
         f"the design allows ± 20 %); no trip at ≤ 12 ms spacing", "R4/C3; if over 30 ms the SW2 budget fails"),
        ("T6", "Contactor release", "as T5; K1 coil current clamp + Q4 drain (TP7); repeat 20×",
         "Q4 off → AUX open; Q4 drain peak", f"release ≤ {el.CONTACTOR['t_release'] * 1e3:.0f} ms; **TP7 ≤ 80 % of Q4's V_DS** (≈ 50 V expected with D1 + Z1: needs the 100 V part, design change 1)",
         "suppression / Q4 rating"),
        ("T7", "Forced discharge one-shot", "armed with the dummy bank; open by heartbeat stop, by each mushroom, by TH1, and **uncommanded** by unplugging the K1 coil",
         "FORCE (TP9) width, TP1/TP2 → 10 V, TP11 K2_DRIVE, TP13 RUN",
         f"FORCE fires on every opening ≤ 1 ms after TP8 opens (simulated {(uv['t7'] - t_far) * 1e3:.1f} ms on the coil unplug), width 100 ms ± 20 %; TP2 (far end) < 10 V "
         f"≤ {T7_LIMIT * 1e3:.0f} ms after TP8 (simulated {uv['t7'] * 1e3:.1f} ms); on the coil unplug TP11 stays low and K2 never re-closes (netlist v4; v3 re-closed it "
         "and fired FORCE only at the 30 ms discrepancy); FORCE never fires while K2_DRIVE is high (T3)", "U6 (Ā = AUX_DB, CLR̄), U28 RUN, U29 debounce"),
        ("T8", "End-to-end stop time", "heartbeat stopped (a hung loop) with the dummy bank charged and a 2.2 Ω load at TP2 standing in for a drive",
         "last heartbeat edge → TP2 < 10 V and load current < 0.5 A (independent of software timestamps)",
         f"≤ {el.SW2_BUDGET * 1e3:.0f} ms in 20/20 runs (simulated worst {max(sc['total'], sc['total_datasheet']) * 1e3:.1f} ms)", "watchdog / release / discharge"),
        ("T9", "No self re-arm", "(a) tap a mushroom for < 50 ms while armed; (b) unplug and replug the K1 coil while armed; (c) restore the heartbeat after T5; (d) supply off/on",
         "LATCH, K1, K2, TP11", "after each: LATCH 0, K1 and K2 stay open until SB1 is pressed again; in (b) TP11 never goes high", "U10 CLR̄ terms, U28"),
        ("T10", "Clamp threshold and hysteresis", "CC supply (no SE-1000) into VBUS through 1 Ω, ramp 24 → 28 V at 1 V/s",
         "TP4 gate on / off levels at TP1", f"on {corners('fix')['nominal'][0]:.2f} V ± 1 %, off {corners('fix')['nominal'][1]:.2f} V ± 1 % (never on below 25.5 V)", "divider / TL431"),
        ("T11", "Clamp switching edge", "regen source 20 A into TP2, clamp chopping", "TP3 drain overshoot, TP4 gate edge, freewheel current",
         f"TP3 ≤ 48 V (simulated {d_m1['v_drain_max']:.1f} V); gate edge 0.2–0.5 µs; no ringing > 5 V after 1 µs", "layout loop / R_G / diode"),
        ("T12", "Regen at the drive", "regen source at TP2: 10 and 20 A steps for 200 ms from the 20 A supply; **30 A = the 20 A supply + the 22 mF bank "
         f"at {bank['v0']:.1f} V through {bank['r']} Ω** ({bank['i0']:.0f} → {bank['i1']:.0f} A extra over {bank['t'] * 1e3:.1f} ms = {bank['cycles']:.0f} chopper cycles at the "
         f"simulated {d045['f_sw_mean'] / 1e3:.1f} kHz; the drive-end voltage settles within the first cycles, so read the 30 A point in that window); "
         "then the bank at 30 V dumped into TP2 10× in 1 s",
         "TP1 and **TP2** max, resistor plate temperature", f"TP2 ≤ {d045['drives']['n0T'] + 0.3:.1f} V at {lim / 26:.0f} A (the {lim:.0f} W regen limit; simulated "
         f"{d045['drives']['n0T']:.2f} V at the lead end, {d045['v_drive_max']:.2f} V at the elbow at {d045['regen_P']:.0f} W); chopper 1–20 kHz, never MHz bursts on TP4; "
         "plate ≤ 60 °C after the 10 dumps; TH1 opens at 85 °C ± 5 (heat gun)", "filter / on-time / regen limit (design changes 0, 5)"),
        ("T13", "Loss of clamp power", "armed, running; pull F2 (12 V / 3.3 V logic) for 1 s", "K1, LATCH, TP4",
         "K1 opens (WD_OK has no supply), board comes back disarmed; with design change 4: 12 V UVLO opens K1 before the 3.3 V rail falls", "UVLO"),
        ("T14", "AUX faults", "(a) cut the AUX opto wire while running; (b) short the AUX opto output while stopped and press ARM; (c) while running, open the aux "
         "loop for 0.1 ms (a MOSFET in series driven by the function generator) 20 times",
         "K1, FORCE, supply current, TP12", f"(a) stop: FORCE ≤ 1 ms after the cut (the AUX_DB edge; simulated {aw * 1e3:.1f} ms), K1 opens ≤ 12 ms (the supply may latch: acceptable); "
         f"(b) ARM refused (design change 2); (c) TP12 never moves, no FORCE, still armed (glitches < {db['gap_reject'] * 1e3:.2f} ms are ignored)", "AUX logic / debounce"),
        ("T15", "Welded K1 (simulated)", "replace K1's main contact with a 10 AWG link (coil still driven), stop the heartbeat",
         "FORCE, supply state", f"design change 3 / 10: FORCE {db['window'][0] * 1e3:.0f}–{db['window'][1] * 1e3:.0f} ms after Q4 off, the SE-1000 latches off (power removed)", "discrepancy monitor"),
        ("T16", "Supply trip", "with the dummy bank and 2.2 Ω, raise the load until the SE-1000 trips (≥ 43.8 A)", "board state after the trip and after re-power",
         "board disarmed after re-power (T1); no fuse operated; wiring ≤ 60 °C", "POR / wiring"),
    ]
    L += [f"| {a} | {b} | {c} | {d} | {e} | {f} |" for a, b, c, d, e, f in T]
    L += ["", "Order: T0–T2 at 1 A current limit; T3–T9 on the SE-1000 with the dummy bank; T10–T12 with the CC regen source; T13–T16 last. "
          "Only then does the first drive (Step 1) get power, with the catch cradle set (electrical.md *Category-0 drop protection*).", ""]
    if B:
        ss1 = el.BASE_SS1
        lo, hi = ss1["t_set"] * (1 - ss1["tol"]), ss1["t_set"] * (1 + ss1["tol"])
        rc = next((r for r in B["regen"] if r["clamp"]), {})
        TB = [
            ("T17", "Base SS1 delay (U20)", "base channel populated, a 1.9 mF dummy bank (4 × 470 µF) + 2.2 Ω on VBUS_B; armed; stop by each mushroom (each of its "
             "channels c, d), by the arm heartbeat, by the base heartbeat (GPIO23); 20 runs", "K1 AUX, Q9 enable, VBUS_B, U20 contacts",
             f"K1 opens ≤ 10 ms on a mushroom while Q9 stays on; Q9 off {lo:.1f}–{hi:.1f} s after the stop (U20 set {ss1['t_set']:.1f} s; the longest software ramp is "
             f"{B['ramp_max']:.1f} s); heartbeats restored during the delay do not cancel it; only ARM re-enables; ARM refused without the base heartbeat",
             "U20 setting / wiring, WD_B"),
            ("T18", "Base clamp leg and FORCE_B", "CC supply into VBUS_B through 1 Ω (ramp 24 → 28 V at 1 V/s); then a T17 stop with the dummy bank charged",
             "Q10 gate on / off at VBUS_B; FORCE_B width; far end of the dummy bank", f"on 26.00 V ± 1 %, off 25.70 V ± 1 %; FORCE_B 100 ms ± 20 % on every Q9 "
             f"opening; dummy bank < 10 V ≤ {max(3.0, 2 * 1e3 * (rc.get('t_uvlo_after_off') or 0.0015)):.0f} ms after Q9 off (simulated "
             f"{_f(rc.get('t_uvlo_after_off') and 1e3 * rc['t_uvlo_after_off'], 1, ' ms')})", "base comparator / U25"),
            ("T19", "Shorted base switch, soft-start, caster feed", "(a) link across Q9, T17 stop; (b) ARM into the empty dummy bank; (c) the caster release "
             f"({el.CASTER_SOL['part']}) on VBUS_B, released, then a T17 stop", "DISC_B, supply current, VBUS_B rise, solenoid current",
             f"(a) DISC_B 50 ms ± 20 % after the enable drops; (b) inrush ≤ {el.BASE_SWITCH['i_limit'] / 3:.0f} A, VBUS_B rises in ≈ {el.BASE_SWITCH['t_soft'] * 1e3:.0f} ms; "
             f"(c) solenoid current ≈ {el.CASTER_SOLENOID_A:.1f} A (ESTIMATE) while released; it drops (all four casters lock) when Q9 opens and never before it while the Pi "
             "holds it released", "LM5069 / DISC_B / caster feed"),
        ]
        L += ["**Robot board only (base channel, §6 of safety_board_sim.md):** before the base is powered; not part of the Step-1 gate (the M1 bench has no base).", "",
              "| # | Test | Setup and stimulus | Measure | Pass | Fail → |", "|---|---|---|---|---|---|"]
        L += [f"| {a} | {b} | {c} | {d} | {e} | {f} |" for a, b, c, d, e, f in TB]
        L.append("")
    L += ["## Current-clamp bandwidth (which clamp each test needs)", "",
          "The sheet asked for ≥ 1 MHz on the 100 A clamp; the BOM's Micsig CP2100A is DC–800 kHz (first-order rise ≈ 0.44 µs, time constant "
          f"{1e6 / (2 * math.pi * CLAMP_BW):.2f} µs). No pass / fail line reads a current edge: edges are timed on voltages (TP3/TP4 with the 100 MHz probe). "
          "What the clamp must read is the amplitude of each current event, which the simulation's own time scales set:", "",
          "| Test | Current read | Fastest feature (simulation) | CP2100A reads | |", "|---|---|---|---|---|"]
    L += [f"| {a} | {b} | {c} | {d} | {'✅' if ok else '⚠️ informational only'} |" for a, b, c, d, ok in clamp_rows]
    L += ["", "**Verdict: 800 kHz is enough; the requirement becomes ≥ 500 kHz.** The only feature it cannot follow is the freewheel diode's µs decay "
          "in T11, which no pass line uses: the freewheel peak equals the leg current at turn-off (the resistor's inductance keeps the current continuous), "
          "and that is read on the ≥ 50 µs flat top. A CP2100B (2.5 MHz, ≈ €699) is not needed.", "",
          ] + review_gate_md() + [
          "## Independent review checklist (schematic + layout, before the board parts are ordered)", "",
          "The reviewer named under *Review gate* above signs each line (name, date) or writes the defect. The reviewer gets the KiCad project, this file, "
          "safety_board_sim.md and electrical.md's netlist; nothing is checked against a description of the board.", "",
          "**Schematic**", "",
          "- [ ] Every net in the electrical.md netlist exists with the same name; every pin of every IC is connected or explicitly tied (no floating CLR̄, PRE, unused gates).",
          "- [ ] SN74LVC1G123 U5 / U6 / one-shots: trigger on the documented pin (Ā falling / B rising), CLR̄ polarity, R·C values give 25 ms / 100 ms (TI SCES586 table, C0G caps); "
          "the CLR̄ ↑ trigger (Ā low, B high) of U6 fires only at a pre-charge abort, as intended (T4).",
          f"- [ ] U10 D = {el.U10_D}; CLR̄ = {el.U10_CLR}; SB1 is the clock, debounced, NO (the same terms as electrical.md's netlist and nets table).",
          "- [ ] **Netlist v4 race fix:** U28 RUN (CLK = AUX_DB ↑, CLR̄ = LATCH); K2_DRIVE = LATCH ∧ ¬RUN ∧ ¬PRE_TIMEOUT; U6 Ā, U8, U10 D, U11, U17 all on AUX_DB, "
          "nothing on AUX_RAW but U29 and GPIO27; no path by which an AUX edge changes U6's CLR̄ while LATCH is high.",
          f"- [ ] **AUX debounce U29:** {el.AUX_DEBOUNCE['parts']}; BAT54 orientation (discharges C_db when AUX_RAW falls); the 74LVC1G17 thresholds at 3.3 V re-derived "
          "from the datasheet and `python -m rnd.safety_board_sim` re-run if they fall outside VT+ 1.3–2.2 V / VT− 0.6–1.5 V.",
          "- [ ] Q4 coil driver V_DS ≥ 2 × (supply + V_Z1) and V_GS(th) specified at the driving voltage; Q5 likewise; gate pull-downs so a dead driver means off.",
          "- [ ] D1 + Z1 orientation (Zener reverse-biased in normal operation, diode blocks the supply); coil suppression on K2 too.",
          "- [ ] Mushroom channel 1 in series with the coil on the **high side** of the coil, channel 2 only to logic; both NC; wiring diagram shows both buttons.",
          "- [ ] TLV3202 divider: 0.1 % resistors, the hysteresis resistor, the 2 µs RC (design change 0); reference TL431B decoupled; PRE_OK compares VBUS with VIN (not with a fixed level).",
          "- [ ] U16 minimum on-time (50 µs, triggered by OV ↑) ORed with OV and FORCE before the UCC27524; nothing else can hold the gate on.",
          f"- [ ] U17 discrepancy monitor: XOR(Q4 gate, AUX_DB), RC {el.DISC_WINDOW['t'] * 1e3:.0f} ms ± {el.DISC_WINDOW['tol']:.0%} (> operate 25 ms + bounce + t_stable = "
          f"{db['need'] * 1e3:.1f} ms; < {el.SW2_BUDGET * 1e3:.0f} ms), into U6's trigger and U10's CLR̄.",
          "- [ ] U18 12 V UVLO and U19 branch-current trip (16 A, 5 s) into U10's CLR̄; F3 still in series for shorts.",
          "- [ ] Robot board: the base branch is fed by Q9 from VIN, **not** from VBUS after K1; Q9's enable only through U20's two OFF-delay contacts in series; "
          "U20's input on the mushrooms' own NC blocks (c, d) + the LATCH photoMOS; WD_B into U10's CLR̄ and D; the caster-release solenoid on VBUS_B; the base clamp leg and "
          "FORCE_B on VBUS_B and nowhere else (no diode from VBUS_B to VBUS).",
          "- [ ] UCC27524 supply from U2 with the 12 V UVLO into the coil chain; gate resistors per MOSFET; Kelvin source not required but source returns to the film cap.",
          "- [ ] Freewheel Schottky across **each** NH-50 (anode at the drain, cathode at VBUS); IFSM, V_RRM ≥ 60 V.",
          "- [ ] Ideal diode U1/Q6 orientation (blocks VBUS → PSU), its gate supply, reverse-recovery behaviour on a fast regen edge.",
          "- [ ] Logic supply U2/U3 from VIN through F2 and nothing else on it; the Pi's ground and the board ground meet at one point; optos on AUX and ESTOP_SENSE.",
          "- [ ] Power-on reset R9·C9 holds LATCH cleared ≥ 50 ms after 3.3 V is valid; brown-out below 2.5 V clears it too.",
          "- [ ] DC ratings: K1 (ALEV100) breaking 100 A at 28 V DC with its polarity marked; K2 contact 10 A DC at 30 V; fuses DC-rated for the pack's short-circuit current (MIDI, ATO).",
          "- [ ] Every rating in electrical.md's tables re-derived from the datasheets by the reviewer (MOSFET SOA at 49 A / 100 ms, NH-50 pulse rating, TVS, capacitor ripple at the chopper frequency).",
          "", "**Layout**", "",
          "- [ ] Clamp loop (film cap → NH-50 leads → diode → MOSFET → source → film cap) area minimised: ≤ 30 nH estimated (≤ 15 mm × 15 mm loop on a 2-layer board, or short bus bars).",
          "- [ ] 146 A path: copper width/thickness or bus bars sized for 49 A per leg pulses and ≈ 15 A continuous; temperature rise ≤ 20 K; 10 AWG lugs crimped, not soldered.",
          "- [ ] High-current return does not share copper with the comparator divider, the TL431 or the logic ground (star point at C1).",
          "- [ ] Divider sense point is the VBUS terminal, Kelvin-connected; trace away from the switching node.",
          "- [ ] Creepage/clearance ≥ 1 mm between VBUS and logic; optos straddle a clear gap.",
          "- [ ] NH-50s on an aluminium plate with TH1 on the same plate; thermal pad; plate grounded or insulated by design.",
          "- [ ] Test points TP1–TP11 accessible with a scope ground spring; current-clamp loops on leg 1, supply lead and K1 coil.",
          "- [ ] Connectors keyed (no swapping the e-stop loop with the heartbeat); strain relief on the 10 AWG leads.",
          "", "**Before sign-off**", "",
          "- [ ] The reviewer re-ran `python -m rnd.safety_board_sim` on the drawn values and every ✅ above still holds; any change goes back into electrical.md.",
          "- [ ] The design changes listed in safety_board_sim.md are in the schematic, or the reviewer accepts the risk in writing.", ""]
    return "\n".join(L)


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--quick", action="store_true")
    a = ap.parse_args(argv)
    R = run(quick=a.quick)
    print(write(R))


if __name__ == "__main__":
    main()
