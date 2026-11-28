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
5. **Logic single faults** (python model of the netlist): reset, brownout / power-on, broken / stuck AUX, welded K1,
   loss of the 12 V clamp supply, watchdog tolerance.

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


def _board(v_in, cmp_corner, force_pwl="0 0", k1_pwl=None, k1_closed=True, ic=25.2, fix=True):
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
        L += [f"VIR{k} vbus r{k}a 0", f"RR{k} r{k}a r{k}b {el.CLAMP['r_each']}", f"LR{k} r{k}b d{k} {el.NH50['L']}",
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
    deck += _board(v_in, corners(variant)[corner], ic=v_in, fix=fix) + hl
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
class Board:
    """Event-driven model of the netlist's logic and relays (0.1 ms steps). Inputs: mushrooms S1/S2 (pressed = open),
    ARM button, heartbeat edges, TH1, logic supply; faults. Signals follow electrical.md's netlist; `fix` adds the changes
    this study proposes (¬AUX on the ARM condition; FORCE also from a K1 that fails to open)."""
    DT = 1e-4

    def __init__(self, which="M1", fix=False, wd_scale=1.0, faults=()):
        self.which, self.fix, self.faults = which, fix, set(faults)
        self.t = 0.0
        self.wd_t = el.WATCHDOG["t_nom"] * wd_scale
        self.s_pressed = False
        self.th1 = True
        self.logic_on = True
        self.latch = False
        self.last_hb = None
        self.k1_coil = self.k2_coil = False
        self.k1 = self.k2 = False                      # contacts
        self.k1_t = self.k2_t = None                   # time of the last coil change
        self.aux_prev = False
        self.force_until = -1.0
        self.pre_t0 = None
        self.pre_timeout = False
        self.vbus = 0.0
        self.vin = el.V_FULL
        self.k1_cmd_off_t = None
        self.log = []
        self.events = []
        self.por_until = 0.1                           # R9·C9 power-on reset holds CLR̄ low

    # signals
    @property
    def aux(self):
        if "aux_broken" in self.faults:
            return False
        if "aux_stuck" in self.faults:
            return True
        return self.k1

    @property
    def wd_ok(self):
        return self.logic_on and self.last_hb is not None and self.t - self.last_hb <= self.wd_t

    @property
    def pre_ok(self):
        return self.vbus >= self.vin - PRE_OK_DV

    @property
    def force(self):
        return self.t < self.force_until and "no_12v" not in self.faults

    def heartbeat(self):
        self.last_hb = self.t

    def press_arm(self):
        if not self.logic_on:
            return False
        d = (not self.s_pressed) and self.wd_ok and self.th1 and (not self.fix or not self.aux)
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

    def _step(self):
        self.t += self.DT
        t = self.t
        stop = self.s_pressed or not self.wd_ok or not self.th1 or not self.logic_on or (self.fix and "no_12v" in self.faults)   # fix 4: 12 V UVLO
        if self.latch and (stop or self.pre_timeout or t < self.por_until or self.force):
            self.latch = False
            self.events.append((t, "LATCH cleared"))
        q4 = self.logic_on and self.latch and self.wd_ok and (self.pre_ok or self.aux)
        coil1 = q4 and not self.s_pressed
        k2_drive = self.logic_on and self.latch and (not self.aux) and (not self.pre_timeout)
        coil2 = k2_drive and not self.s_pressed
        if coil1 != self.k1_coil:
            self.k1_coil, self.k1_t = coil1, t
            if not coil1:
                self.k1_cmd_off_t = t
        if coil2 != self.k2_coil:
            self.k2_coil, self.k2_t = coil2, t
        if "k1_welded" not in self.faults:
            if self.k1_coil and not self.k1 and t - self.k1_t >= T_K1_OPERATE[1]:
                self.k1 = True
                self.events.append((t, "K1 closed"))
            if not self.k1_coil and self.k1 and t - self.k1_t >= el.CONTACTOR["t_release"]:
                self.k1 = False
                self.events.append((t, "K1 open"))
        elif self.k1_coil and not self.k1 and t - self.k1_t >= T_K1_OPERATE[1]:
            self.k1 = True
        if self.k2_coil and not self.k2 and t - self.k2_t >= 0.010:
            self.k2 = True
        if not self.k2_coil and self.k2 and t - self.k2_t >= T_K2_RELEASE:
            self.k2 = False
        aux = self.aux
        if self.aux_prev and not aux and not k2_drive:         # U6: Ā falling edge, CLR̄ = ¬K2_DRIVE
            self.force_until = t + el.T_FORCE
            self.events.append((t, "FORCE"))
        if self.fix:                                          # fix 3: K1 discrepancy monitor, coil command ≠ AUX for > 30 ms
            if self.k1_coil != aux:
                self.disc_t = t if getattr(self, "disc_t", None) is None else self.disc_t
                if t - self.disc_t > 0.030 and self.force_until < t:
                    self.force_until = t + el.T_FORCE
                    self.latch = False
                    self.events.append((t, "FORCE (K1 discrepancy)"))
                    self.disc_t = None
            else:
                self.disc_t = None
        self.aux_prev = aux
        if self.latch and self.pre_t0 is not None and not aux and t - self.pre_t0 > el.T_PRE_TIMEOUT:
            self.pre_timeout = True
        # bus (lumped RC): the supply through K1 (stiff) or the pre-charge resistor, the clamp when forced or over its
        # threshold, the drives' idle draw and the bleeder
        r_cl = el.CLAMP["r_each"] / 3
        clamp = (self.force or self.vbus > corners("fix")["nominal"][0]) and "no_12v" not in self.faults
        if self.k1:
            self.vbus = self.vin * (r_cl / (r_cl + 0.02) if clamp else 1.0)
        else:
            i = (self.vin - self.vbus) / el.R_PRE if self.k2 else 0.0
            i -= (self.vbus / r_cl if clamp else 0.0) + el.N_DRIVERS[self.which] * el.I_IDLE_DRIVER * self.vbus / el.V_FULL + self.vbus / el.R_BLEED
            self.vbus = max(self.vbus + self.DT * i / el.c_bus(self.which), 0.0)
        self.log.append((t, self.latch, self.k1, self.k2, self.force, self.vbus))


def _run_hb(b, secs, hb=True, period=0.002):
    n = int(round(secs / b.DT))
    k = int(round(period / b.DT))
    for i in range(n):
        if hb and i % k == 0:
            b.heartbeat()
        b.step()


def logic_scenarios():
    """Each scenario: the netlist as drawn (v3) and with this study's fixes."""
    res = []

    def arm_and_run(b):
        _run_hb(b, 0.15)
        b.press_arm()
        _run_hb(b, 0.3)
        return b

    for fix in (False, True):
        tag = "v3 + fixes" if fix else "v3 as drawn"
        # 1. normal arm → run → watchdog loss → stop, no self re-arm
        b = arm_and_run(Board(fix=fix))
        ok_run = b.k1 and not b.k2
        t0 = b.t
        _run_hb(b, 0.3, hb=False)
        k1_open = next((t for t, e in b.events if e == "K1 open" and t > t0), None)
        forced = any(e.startswith("FORCE") and t > t0 for t, e in b.events)
        _run_hb(b, 0.5)                        # heartbeat back: must stay off until ARM
        res.append(dict(case="heartbeat stops while running", variant=tag, ok=ok_run and k1_open is not None and forced and not b.k1 and not b.latch,
                        detail=f"K1 open {1e3 * (k1_open - t0):.1f} ms after the last edge (wd {b.wd_t * 1e3:.0f} ms + release), FORCE {'yes' if forced else 'NO'}, "
                               f"heartbeat back → K1 {'closed (self re-arm!)' if b.k1 else 'stays open'}"))
        # 2. ARM refused with a mushroom pressed, or without a heartbeat
        b = Board(fix=fix)
        _run_hb(b, 0.15)
        b.s_pressed = True
        r1 = b.press_arm()
        b.s_pressed = False
        b2 = Board(fix=fix)
        b2.step(1500)
        r2 = b2.press_arm()
        res.append(dict(case="ARM with a mushroom pressed / without heartbeat", variant=tag, ok=not r1 and not r2, detail=f"LATCH {r1} / {r2}"))
        # 3. brownout: logic supply dips (supply trip) and returns
        b = arm_and_run(Board(fix=fix))
        b.power(False)
        _run_hb(b, 0.05, hb=False)
        b.power(True)
        _run_hb(b, 0.5)
        res.append(dict(case="logic supply lost and restored (PSU trip, brownout)", variant=tag, ok=not b.k1 and not b.latch,
                        detail=f"after power returns: LATCH {b.latch}, K1 {'closed' if b.k1 else 'open'} (power-on reset)"))
        # 4. coil interrupted while armed (K1 drops for another reason)
        b = arm_and_run(Board(fix=fix))
        b.s_pressed = True
        b.step(5)
        b.s_pressed = False
        _run_hb(b, 0.4)
        res.append(dict(case="mushroom tapped (coil loop broken for 0.5 ms) while armed", variant=tag, ok=not b.k1 and not b.latch,
                        detail=f"LATCH {b.latch}, K1 {'closed' if b.k1 else 'open'}"))
        # 5. broken AUX wire while running
        b = arm_and_run(Board(fix=fix))
        b.faults.add("aux_broken")
        t0 = b.t
        _run_hb(b, 0.3)
        f = [t for t, e in b.events if e.startswith("FORCE") and t > t0]
        k1o = next((t for t, e in b.events if e == "K1 open" and t > t0), None)
        res.append(dict(case="AUX wire breaks while running", variant=tag, ok=not b.k1,
                        detail=f"discrepancy → FORCE at +{1e3 * (f[0] - t0):.0f} ms and LATCH cleared, K1 open at +{1e3 * (k1o - t0):.0f} ms (the clamp "
                               f"loads the supply for the release time: the bench supply may latch); stop, fail-safe"
                        if f and k1o else "no FORCE (U6 is held cleared: K2_DRIVE = LATCH ∧ ¬AUX goes high and K2 re-closes), K1 stays closed on PRE_OK: "
                                          "a latent fault; the next stop gets **no forced discharge** (≈ 300 ms of bus on the drives' idle draw). "
                                          "Only the Pi sees it (AUX_OK low → supervisor e-stop latch)"))
        # 6. AUX stuck closed (welded aux, shorted opto) before an ARM
        b = Board(fix=fix, faults=("aux_stuck",))
        _run_hb(b, 0.15)
        b.press_arm()
        _run_hb(b, 0.1)
        k1_no_pre = b.k1 and max(v for *_, v in b.log[:int(0.15 / b.DT) + 300] or [(0, 0, 0, 0, 0, 0)]) < 1.0
        vb_at_close = next((b.log[i - 1][5] for i, lg in enumerate(b.log) if lg[2] and i), None)
        res.append(dict(case="AUX stuck closed, then ARM", variant=tag, ok=not b.latch,
                        detail=("K1 closes on an empty bus (pre-charge bypassed: Q4 holds on AUX, K2_DRIVE = ¬AUX is off), "
                                f"bus {0.0 if vb_at_close is None else vb_at_close:.1f} V at closing → inrush limited only by ESR / wiring; "
                                "and no aux edge at the next stop, so no FORCE") if b.latch else "ARM refused (¬AUX on the ARM condition)"))
        # 7. K1 welded: stop requested
        b = arm_and_run(Board(fix=fix))
        b.faults.add("k1_welded")
        t0 = b.t
        _run_hb(b, 0.3, hb=False)
        f = [t for t, e in b.events if e.startswith("FORCE") and t > t0]
        res.append(dict(case="K1 main contacts welded, heartbeat stops", variant=tag, ok=bool(f),
                        detail=(f"FORCE {1e3 * (f[0] - t0):.0f} ms after the last edge: the clamp loads the supply at "
                                f"{el.V_FULL / (el.CLAMP['r_each'] / 3):.0f} A → the bench supply latches off (≥ 43.8 A); on the robot the pack "
                                "keeps the bus up: needs a second break (BMS discharge FET off)") if f else
                               "no aux edge → no FORCE; the bus stays powered from the supply; only the Pi (DISABLE frames) and the drives' CAN timeout remain"))
        # 8. 12 V clamp supply lost (U2 dead) while running and regenerating
        b = arm_and_run(Board(fix=fix, faults=("no_12v",)))
        res.append(dict(case="12 V gate-driver supply lost (U2 / F2) while running", variant=tag, ok=not b.k1,
                        detail=("12 V UVLO → STOP: K1 opens, LATCH cleared (the bus then has no clamp: the 1 kΩ bleeder and the SMDJ33A only)" if fix else
                                "the comparator still switches but UCC27524 has no supply: no clamp, no FORCE, and K1 stays closed. Regen then charges the bus "
                                f"without limit (M1 1.3 kW into 7 mF: 25 → 28 V in ≈ {1e3 * 7.05e-3 * (28 ** 2 - 25.2 ** 2) / 2 / 1328:.0f} ms)")))
    return res


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
    R["logic"] = logic_scenarios()
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
         "electrical.md netlist (v3). Astra review #3, critical 3: *simulate the board before power*. A model of an unbuilt board: "
         "it decides what to build and what the bench measures ([safety_board_tests.md](safety_board_tests.md)); it is not evidence the board works.", "",
         "## 1. Regen clamp dump: bus overshoot at the drives", "",
         "Board VBUS (C1 4.7 mF, C2 10 µF film, SMDJ33A, 1 kΩ bleeder, 3 × NH-50 0.56 Ω + 1 µH, VS-40CPQ060 freewheel, IRFB7530 VDMOS, "
         f"R_G {R_GATE} Ω, UCC27524, TLV3202 as a hysteretic switch with its delay) → harness (fuse, XT60s, 14/16 AWG pairs at 60 °C, "
         f"{L_PAIR * 1e6:.1f} µH/m) → each drive's 470 µF + its regen as P(t)/V. Corners: ± 1 % on both thresholds (TL431B + 0.1 % divider). "
         f"Variants: *v3 as drawn* (on {VARIANTS['v3']['rise']:.1f} / off {VARIANTS['v3']['fall']:.1f} V, comparator straight on the divider); *proposed* (on {VARIANTS['fix']['rise']:.1f} / off {VARIANTS['fix']['fall']:.1f} V, {CMP_FILTER * 1e6:.0f} µs divider filter, {T_ON_MIN * 1e6:.0f} µs minimum on-time).", "",
         "| Board | Case | Regen peak / energy (run) | Clamp on / off at VBUS | **Highest drive** (node) | Lowest drive | Drain peak | Leg current | Energy per NH-50 | Chopper f | |",
         "|---|---|---|---|---|---|---|---|---|---|---|"]
    vname = {"v3": "v3 as drawn", "v3+ton": "v3 thresholds + filter + on-time", "fix": "proposed"}
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
    L += ["## 5. Logic single faults (python model of the netlist)", "", "| Scenario | Netlist | Result | |", "|---|---|---|---|"]
    L += [f"| {r['case']} | {r['variant']} | {r['detail']} | {'✅' if r['ok'] else '❌'} |" for r in R["logic"]]
    L += ["", "## Design changes from this simulation", ""] + DESIGN_CHANGES + [""]
    return "\n".join(L)


DESIGN_CHANGES = [
    "0. **Clamp comparator: 2 µs RC on the divider + a 50 µs minimum on-time** (one more SN74LVC1G123 on OV ↑, ORed with OV and FORCE before the "
    "UCC27524). As drawn the clamp's own turn-on step at the board node is larger than its hysteresis and it rings at MHz (section 1). "
    "**Thresholds: on at 26.0 V, off at 25.7 V** (was 27.0 / 25.8 V as the corners were computed): ± 1 % still clears a full pack and every drive gains ≈ 1 V.",
    "1. **Q4 (K1 coil driver) → 100 V logic-level MOSFET** (e.g. an IRL540N-class part, V_GS(th) ≤ 2 V, R_DS(on) specified at 4–5 V; drive it from "
    "the UCC27524's spare channel at 12 V if no 100 V part is specified at 3.3 V). With D1 + Z1 across the coil the drain sees supply + V_Z + V_F ≈ 50 V at "
    "every turn-off; the IRLB8721 is a 30 V part. Q5 (K2) likewise ≥ 60 V.",
    "2. **ARM needs AUX open:** U10's D input becomes S1b ∧ S2b ∧ WD_OK ∧ TH1 ∧ ¬AUX. A stuck-closed AUX (welded aux, shorted opto) otherwise lets K1 "
    "close straight onto an empty bus (Q4 holds on AUX, K2_DRIVE = ¬AUX never pre-charges) and removes the opening edge that fires FORCE.",
    "3. **K1 discrepancy monitor:** coil command (Q4 gate) ≠ AUX for > 30 ms (longer than the 25 ms operate time) fires FORCE and clears LATCH (an XOR, "
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
]


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
    L = ["# Safety board: motorless bench tests (generated by `python -m rnd.safety_board_sim`)", "",
         "**Step-1 gate (M1_READINESS exit criterion 1):** every test below passes on the built board, with no motor connected, before any drive is "
         "powered from it. Numbers come from the simulation ([safety_board_sim.md](safety_board_sim.md)) with tolerances; a reading outside the pass band "
         "stops the build until it is understood. Record every scope capture (PNG + CSV) in docs/sw/board_tests/ with the date and the board revision.", "",
         "## Equipment", "",
         "- Bench supply: the SE-1000-24 set to 25.2 V (and the 0–30 V / 10 A CC supply for T0 and T9); a second CC supply (0–30 V) for the clamp tests.",
         "- **Dummy drive load:** a capacitor bank of 2.35 mF (5 × 470 µF 35 V, the M1 drivers' input capacitance) on 0.5 m of 14 AWG at the end of the real "
         "1.5 m arm lead + F3, plus switchable resistors: 10 Ω 50 W (RH-50, pre-charge fault), 5.6 Ω and 2.2 Ω 50 W on the heatsink.",
         "- **Regen source:** the CC supply in series with a 1N5822-class diode into the dummy drive bank (it can push current *into* the bus, which the "
         "SE-1000 cannot sink); a 30 V / 20 A CC supply gives ≈ 500 W of simulated regen. For energy tests: a 22 mF 50 V bank charged to 30 V (9.9 J … "
         "repeat 10× in 1 s for the pulse duty).",
         "- 4-channel scope (≥ 100 MHz, 1 GS/s) with a ≥ 100 V differential probe, a 100 A current clamp (DC, ≥ 1 MHz) and a 30 A clamp; a function generator "
         "(3.3 V square, 250 Hz = the 500 Hz heartbeat toggle) for GPIO17; a thermocouple on the resistor plate; DMM; a stopwatch log (the scope timebase is "
         "the reference, not any software timestamp).",
         "- Scope points: **TP1 VBUS at the board**, **TP2 VBUS at the dummy drive bank** (end of the arm lead), **TP3 Q1 drain** (diff probe, shortest ground), "
         "**TP4 Q1 gate**, TP5 WD_OK, TP6 Q4 gate, **TP7 Q4 drain**, TP8 AUX (opto output), TP9 FORCE, TP10 LATCH, TP11 K2_DRIVE; current: clamp leg 1, supply lead, K1 coil.", "",
         "## Tests (pass / fail)", "",
         "| # | Test | Setup and stimulus | Measure | Pass | Fail → |", "|---|---|---|---|---|---|"]
    T = [
        ("T0", "First power, logic only", "CC supply at 25.2 V / 1 A limit, no load, no heartbeat", "supply current; 12 V and 3.3 V rails; LATCH",
         "≤ 0.15 A; 12 V ± 5 %, 3.3 V ± 3 %; LATCH 0, K1 and K2 open", "stop: rails / shorts"),
        ("T1", "Power-on comes up disarmed", "supply off → on 10 times, heartbeat running, ARM not pressed; then 5 times with ARM held down during power-up",
         "LATCH, K1, K2 (TP10, AUX)", "LATCH stays 0 every time (R9·C9 POR ≥ 50 ms on TP10)", "POR network"),
        ("T2", "ARM refused", "(a) a mushroom pressed (each one, each channel); (b) no heartbeat; (c) TH1 open (unplug); (d) **AUX forced high** (short the opto output) with the fix",
         "LATCH after SB1", "LATCH stays 0 in every case", "U10 D gating"),
        ("T3", "Pre-charge into the dummy bank", f"heartbeat on, ARM; dummy bank 2.35 mF + board 4.7 mF; measure the real idle draw first (drives' logic boards if available)",
         "TP1/TP2 rise, K2 / K1 timing, supply current", f"PRE_OK at {p_m1['t_pre_ok'] * 1e3:.0f} ms ± 30 % (τ = {tau_pre * 1e3:.0f} ms); inrush ≤ {el.V_FULL / el.R_PRE * 1.1:.1f} A; "
         f"K1 closes ≤ {(p_m1['t_k1']) * 1e3 + 10:.0f} ms after ARM with ΔV ≤ {max(p_m1['dv_at_close'], 0.5):.1f} V; K2 open ≤ 10 ms after AUX; idle draw × 3.3 Ω ≤ 2.0 V",
         "pre-charge / PRE_OK divider"),
        ("T4", "Pre-charge fault", "10 Ω RH-50 across VBUS, ARM", "TP1, LATCH, K2, R3 temperature",
         f"bus settles ≈ {p_dum['v_final']:.1f} V (< VIN − 3 V), K1 never closes, PRE_TIMEOUT clears LATCH at 1.0 s ± 0.2 s, K2 opens; R3 ≤ {p_dum['e_r3'] * 1.2:.0f} J",
         "U11 timer"),
        ("T5", "Watchdog timeout", "armed, running; stop the 250 Hz square at a random phase (20 runs); also 3 runs each at 4 / 8 / 12 ms edge spacing (must not trip)",
         "last edge → WD_OK low (TP5) → Q4 gate (TP6) → AUX (TP8)", f"WD_OK low {wd * 1e3 * 0.9:.1f}–{wd * 1e3 * 1.2:.1f} ms after the last edge (25 ms ± 10 % with a C0G cap; "
         f"the design allows ± 20 %); no trip at ≤ 12 ms spacing", "R4/C3; if over 30 ms the SW2 budget fails"),
        ("T6", "Contactor release", "as T5; K1 coil current clamp + Q4 drain (TP7); repeat 20×",
         "Q4 off → AUX open; Q4 drain peak", f"release ≤ {el.CONTACTOR['t_release'] * 1e3:.0f} ms; **TP7 ≤ 80 % of Q4's V_DS** (≈ 50 V expected with D1 + Z1: needs the 100 V part, design change 1)",
         "suppression / Q4 rating"),
        ("T7", "Forced discharge one-shot", "armed with the dummy bank; open by heartbeat stop, by each mushroom, by TH1, by unplugging the K1 coil",
         "FORCE (TP9) width, TP1/TP2 → 10 V", f"FORCE fires on every opening, width 100 ms ± 20 %; TP2 (far end) < 10 V ≤ {max(dis['t_uvlo_far'] or 0, 0.003) * 1e3 * 1.5:.0f} ms after AUX; "
         "FORCE never fires during T3 / T4 (pre-charge)", "U6 wiring (Ā, CLR̄)"),
        ("T8", "End-to-end stop time", "heartbeat stopped (a hung loop) with the dummy bank charged and a 2.2 Ω load at TP2 standing in for a drive",
         "last heartbeat edge → TP2 < 10 V and load current < 0.5 A (independent of software timestamps)",
         f"≤ {el.SW2_BUDGET * 1e3:.0f} ms in 20/20 runs (simulated worst {max(sc['total'], sc['total_datasheet']) * 1e3:.1f} ms)", "watchdog / release / discharge"),
        ("T9", "No self re-arm", "(a) tap a mushroom for < 50 ms while armed; (b) unplug and replug the K1 coil while armed; (c) restore the heartbeat after T5; (d) supply off/on",
         "LATCH, K1, K2", "after each: LATCH 0, K1 and K2 stay open until SB1 is pressed again", "U10 CLR̄ terms"),
        ("T10", "Clamp threshold and hysteresis", "CC supply (no SE-1000) into VBUS through 1 Ω, ramp 24 → 28 V at 1 V/s",
         "TP4 gate on / off levels at TP1", f"on {corners('fix')['nominal'][0]:.2f} V ± 1 %, off {corners('fix')['nominal'][1]:.2f} V ± 1 % (never on below 25.5 V)", "divider / TL431"),
        ("T11", "Clamp switching edge", "regen source 20 A into TP2, clamp chopping", "TP3 drain overshoot, TP4 gate edge, freewheel current",
         f"TP3 ≤ 48 V (simulated {d_m1['v_drain_max']:.1f} V); gate edge 0.2–0.5 µs; no ringing > 5 V after 1 µs", "layout loop / R_G / diode"),
        ("T12", "Regen at the drive", "regen source at TP2: 10, 20, 30 A steps for 200 ms; then the 22 mF / 30 V bank dumped into TP2 10× in 1 s",
         "TP1 and **TP2** max, resistor plate temperature", f"TP2 ≤ {d045['drives']['n0T'] + 0.3:.1f} V at {lim / 26:.0f} A (the {lim:.0f} W regen limit; simulated "
         f"{d045['drives']['n0T']:.2f} V at the lead end, {d045['v_drive_max']:.2f} V at the elbow at {d045['regen_P']:.0f} W); chopper 1–20 kHz, never MHz bursts on TP4; "
         "plate ≤ 60 °C after the 10 dumps; TH1 opens at 85 °C ± 5 (heat gun)", "filter / on-time / regen limit (design changes 0, 5)"),
        ("T13", "Loss of clamp power", "armed, running; pull F2 (12 V / 3.3 V logic) for 1 s", "K1, LATCH, TP4",
         "K1 opens (WD_OK has no supply), board comes back disarmed; with design change 4: 12 V UVLO opens K1 before the 3.3 V rail falls", "UVLO"),
        ("T14", "AUX faults", "(a) cut the AUX opto wire while running; (b) short the AUX opto output while stopped and press ARM",
         "K1, FORCE, supply current", "(a) stop (FORCE + K1 opens; the supply may latch: acceptable); (b) ARM refused (design change 2)", "AUX logic"),
        ("T15", "Welded K1 (simulated)", "replace K1's main contact with a 10 AWG link (coil still driven), stop the heartbeat",
         "FORCE, supply state", "design change 3: FORCE 30–40 ms after Q4 off, the SE-1000 latches off (power removed)", "discrepancy monitor"),
        ("T16", "Supply trip", "with the dummy bank and 2.2 Ω, raise the load until the SE-1000 trips (≥ 43.8 A)", "board state after the trip and after re-power",
         "board disarmed after re-power (T1); no fuse operated; wiring ≤ 60 °C", "POR / wiring"),
    ]
    L += [f"| {a} | {b} | {c} | {d} | {e} | {f} |" for a, b, c, d, e, f in T]
    L += ["", "Order: T0–T2 at 1 A current limit; T3–T9 on the SE-1000 with the dummy bank; T10–T12 with the CC regen source; T13–T16 last. "
          "Only then does the first drive (Step 1) get power, with the catch cradle set (electrical.md *Category-0 drop protection*).", "",
          "## Independent review checklist (schematic + layout, before the board parts are ordered)", "",
          "A person who did not draw the board signs each line (name, date) or writes the defect. The reviewer gets the KiCad project, this file, "
          "safety_board_sim.md and electrical.md's netlist; nothing is checked against a description of the board.", "",
          "**Schematic**", "",
          "- [ ] Every net in the electrical.md netlist exists with the same name; every pin of every IC is connected or explicitly tied (no floating CLR̄, PRE, unused gates).",
          "- [ ] SN74LVC1G123 U5 / U6 / one-shots: trigger on the documented pin (Ā falling / B rising), CLR̄ polarity, R·C values give 25 ms / 100 ms (TI SCES586 table, C0G caps).",
          "- [ ] U10 D = S1b ∧ S2b ∧ WD_OK ∧ TH1 ∧ ¬AUX; CLR̄ = ¬(STOP ∨ FORCE ∨ POR ∨ PRE_TIMEOUT); SB1 is the clock, debounced, NO.",
          "- [ ] Q4 coil driver V_DS ≥ 2 × (supply + V_Z1) and V_GS(th) specified at the driving voltage; Q5 likewise; gate pull-downs so a dead driver means off.",
          "- [ ] D1 + Z1 orientation (Zener reverse-biased in normal operation, diode blocks the supply); coil suppression on K2 too.",
          "- [ ] Mushroom channel 1 in series with the coil on the **high side** of the coil, channel 2 only to logic; both NC; wiring diagram shows both buttons.",
          "- [ ] TLV3202 divider: 0.1 % resistors, the hysteresis resistor, the 2 µs RC (design change 0); reference TL431B decoupled; PRE_OK compares VBUS with VIN (not with a fixed level).",
          "- [ ] U16 minimum on-time (50 µs, triggered by OV ↑) ORed with OV and FORCE before the UCC27524; nothing else can hold the gate on.",
          "- [ ] U17 discrepancy monitor: XOR(Q4 gate, AUX), RC 30 ms (> the contactor's 25 ms max operate time), into U6's trigger and U10's CLR̄.",
          "- [ ] U18 12 V UVLO and U19 branch-current trip (16 A, 5 s) into U10's CLR̄; F3 still in series for shorts.",
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
