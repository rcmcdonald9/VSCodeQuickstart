"""Safety board netlist v4 (rnd/electrical.py) → KiCad 7 project in hardware/safety_board/.

    python -m rnd.kicad_board            # schematic, netlist, PDF, BOM, model JSON, netlist ERC
    python -m rnd.kicad_board --pcb      # + the board (rnd/kicad_pcb.py under /usr/bin/python3, which has pcbnew)

Source of truth: rnd/electrical.py. Every netlist-v4 part comes from the ref table that `safety_board_lines()` prints
(electrical.md *Safety board*); values come from electrical.py's constants (watchdog / one-shot R·C, debounce, clamp
window and filter, pre-charge and bleeder resistors, bulk C, fuses) or, where electrical.py has no constant, from that
table's own text, checked by `need()` so a change in electrical.py stops the generator instead of drifting. The
electrical.md netlist names functions, not pins; the pin-level circuit (which pin of which gate, the glue gates that
build its boolean terms, pull-downs, decoupling, sense dividers) is drawn here and every addition is marked
`Origin = added` with the reason, so the G-SB reviewer can tell netlist from drawing (docs/rnd/safety_board_kicad.md).

The schematic is written as S-expressions (no GUI): each symbol pin gets a short wire stub and a net label, so the
connectivity is exactly the model's. rnd/kicad_check.py compares the exported netlist with the model and with
electrical.py's table.
"""
from __future__ import annotations

import csv
import json
import math
import re
import subprocess
import sys
import uuid
from dataclasses import dataclass, field
from pathlib import Path

from . import electrical as el
from .kicad_sexp import Lib, dump
from .kicad_symbols import library_text

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "hardware" / "safety_board"
KSYM = Path("/usr/share/kicad/symbols")
KFP = Path("/usr/share/kicad/footprints")
PROJECT = "safety_board"
PLIB = "pip_safety"
NS = uuid.UUID("6c1f3b52-4e0b-4d55-9f0e-7d5a1c2b9e01")


def uid(*parts):
    return str(uuid.uuid5(NS, "/".join(str(p) for p in parts)))


# ------------------------------------------------------------------ electrical.py netlist access (drift guards)
def netlist_rows():
    """The ref table of electrical.md *Safety board* as {ref cell: (part, nets, function)}."""
    L = el.safety_board_lines()
    i = L.index("| Ref | Part | Nets | Function |")
    rows = {}
    for line in L[i + 2:]:
        if not line.startswith("| "):
            break
        cells = [c.strip() for c in line.strip("|").split(" | ")]
        rows[cells[0]] = tuple(cells[1:4])
    return rows


def netlist_art():
    L = el.safety_board_lines()
    a = L.index("```")
    b = L.index("```", a + 1)
    return L[a + 1:b]


ROWS = netlist_rows()
ART = "\n".join(netlist_art())


def need(row, *subs):
    """Assert that electrical.py's table row still says what the drawing assumes; return the row text."""
    txt = " | ".join(ROWS[row])
    for s in subs:
        if s not in txt:
            raise SystemExit(f"electrical.py netlist drift: row {row!r} no longer contains {s!r}; update rnd/kicad_board.py")
    return txt


def need_art(*subs):
    for s in subs:
        if s not in ART:
            raise SystemExit(f"electrical.py netlist drift: block diagram no longer contains {s!r}; update rnd/kicad_board.py")


# ------------------------------------------------------------------ values
E96 = [1.00, 1.02, 1.05, 1.07, 1.10, 1.13, 1.15, 1.18, 1.21, 1.24, 1.27, 1.30, 1.33, 1.37, 1.40, 1.43, 1.47, 1.50, 1.54,
       1.58, 1.62, 1.65, 1.69, 1.74, 1.78, 1.82, 1.87, 1.91, 1.96, 2.00, 2.05, 2.10, 2.15, 2.21, 2.26, 2.32, 2.37, 2.43,
       2.49, 2.55, 2.61, 2.67, 2.74, 2.80, 2.87, 2.94, 3.01, 3.09, 3.16, 3.24, 3.32, 3.40, 3.48, 3.57, 3.65, 3.74, 3.83,
       3.92, 4.02, 4.12, 4.22, 4.32, 4.42, 4.53, 4.64, 4.75, 4.87, 4.99, 5.11, 5.23, 5.36, 5.49, 5.62, 5.76, 5.90, 6.04,
       6.19, 6.34, 6.49, 6.65, 6.81, 6.98, 7.15, 7.32, 7.50, 7.68, 7.87, 8.06, 8.25, 8.45, 8.66, 8.87, 9.09, 9.31, 9.53, 9.76]


def e96(x):
    d = 10 ** math.floor(math.log10(x))
    return min((m * d for m in E96 + [10.0]), key=lambda v: abs(math.log(v / x)))


def fr(ohm):
    """resistor value text: 250k, 4.7k, 0.56, 1M, 10.7k"""
    for k, s in ((1e6, "M"), (1e3, "k")):
        if ohm >= k:
            v = ohm / k
            return f"{v:.3g}{s}"
    return f"{ohm:.3g}"


def fc(f):
    for k, s in ((1e-3, "m"), (1e-6, "u"), (1e-9, "n"), (1e-12, "p")):
        if f >= k * 0.999:
            return f"{f / k:.3g}{s}"
    return f"{f:g}"


VREF = 2.495               # TL431B reference (TI SLVS543: 2.495 V typ, ± 0.5 % B grade)
LVC_K = 1.0                # SN74LVC1G123 t_w ≈ K·R·C, K ≈ 1 (TI SLVA720), as electrical.py
VDD = 3.3


def _ov(v_on, v_off, r_top):
    r_h = e96(r_top * VDD / (v_on - v_off))
    gt, gh = 1 / r_top, 1 / r_h
    r_b = e96(1 / (gt * (v_on / VREF - 1) - gh))
    gb = 1 / r_b
    on = VREF * (gt + gb + gh) / gt
    off = (VREF * (gt + gb + gh) - VDD * gh) / gt
    return dict(r_top=r_top, r_bot=r_b, r_hys=r_h, on=on, off=off, r_th=1 / (gt + gb + gh))


def ov_divider(v_on, v_off):
    """Comparator + node: R_top from the bus, R_bot to GND, R_hys from the push-pull output (0 / 3.3 V).
    Off-state (out = 0): trips at v_on; on-state (out = 3.3 V): releases at v_off. E96 values, best of R_top 80-120k."""
    cands = [_ov(v_on, v_off, m * 1e5) for m in E96 if 0.8 <= m <= 1.25] + [_ov(v_on, v_off, m * 1e4) for m in E96 if m >= 8.0]
    return min(cands, key=lambda d: abs(d["on"] - v_on) + abs(d["off"] - v_off))


def preok_network(offset=3.0, k=0.1, ra=100e3):
    """PRE_OK = VBUS ≥ VIN − offset: + node = k·VBUS + VREF·Gb/Gtot (Ra from VBUS, Rb from VREF, Rc to GND),
    − node = k·VIN (Rd / Re)."""
    rb = e96(ra * VREF / offset)
    ga, gb = 1 / ra, 1 / rb
    gc = ga / k - ga - gb
    rc = e96(1 / gc)
    gtot = ga + gb + 1 / rc
    kk = ga / gtot
    rd = 100e3
    re_ = e96(rd * kk / (1 - kk))
    km = re_ / (rd + re_)
    off_real = (VREF * gb / gtot) / km            # VBUS·kk + VREF·gb/gtot > VIN·km → with kk ≈ km
    return dict(ra=ra, rb=rb, rc=rc, rd=rd, re=re_, k_plus=kk, k_minus=km, offset=off_real)


def ratio_ref(frac, r_top=10e3):
    rb = e96(r_top * frac / (1 - frac))
    return r_top, rb, rb / (r_top + rb)


# ------------------------------------------------------------------ the model
@dataclass
class Part:
    ref: str
    lib: str                     # "Device:R"
    value: str
    fp: str                      # footprint lib:name, "" for chassis parts
    sheet: str
    pins: dict                   # pin number → net (None = no connect)
    origin: str = "netlist v4"   # or "added: <why>"
    nl: str = ""                 # electrical.py ref this part implements (block ref)
    loc: str = "board"           # board / chassis
    variant: str = "all"         # all / robot (base channel: not populated on the M1 bench) / M1
    basis: str = ""              # where the value comes from
    group: str = ""              # schematic block title
    units: list = field(default_factory=lambda: [1])


PARTS: list[Part] = []
NOTES: dict[str, list[str]] = {}
VALUES: list[tuple] = []         # (quantity, electrical.py, drawn) for the cross-check report
_GROUP = [""]
_SHEET = [""]


def sheet(name):
    _SHEET[0] = name


def group(title):
    _GROUP[0] = title


def P(ref, lib, value, fp, pins, **kw):
    kw.setdefault("group", _GROUP[0])
    p = Part(ref, lib, value, fp, kw.pop("sheet", _SHEET[0]), {str(k): v for k, v in pins.items()}, **kw)
    if any(x.ref == ref for x in PARTS):
        raise SystemExit(f"duplicate reference {ref}")
    PARTS.append(p)
    return p


R0603, R1206, R2512 = "Resistor_SMD:R_0603_1608Metric", "Resistor_SMD:R_1206_3216Metric", "Resistor_SMD:R_2512_6332Metric"
R3W = "Resistor_THT:R_Axial_Power_L20.0mm_W6.4mm_P22.40mm"
C0603, C1206, C1210 = "Capacitor_SMD:C_0603_1608Metric", "Capacitor_SMD:C_1206_3216Metric", "Capacitor_SMD:C_1210_3225Metric"
FILM10U = "Capacitor_THT:C_Rect_L31.5mm_W11.0mm_P27.50mm_MKS4"
SOT235, SOT236, SOT23, VSSOP8, SOIC8 = ("Package_TO_SOT_SMD:SOT-23-5", "Package_TO_SOT_SMD:SOT-23-6", "Package_TO_SOT_SMD:SOT-23",
                                        "Package_SO:VSSOP-8_2.3x2mm_P0.5mm", "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm")
TO220, TO247 = "Package_TO_SOT_THT:TO-220-3_Vertical", "Package_TO_SOT_THT:TO-247-3_Vertical"
STUD5, STUD4 = "MountingHole:MountingHole_5.3mm_M5_Pad_Via", "MountingHole:MountingHole_4.3mm_M4_Pad_Via"
ATO = "Fuse:Fuseholder_Blade_ATO_Littelfuse_Pudenz_2_Pin"
TPFP = "TestPoint:TestPoint_Keystone_5005-5009_Compact"
BAT54 = "Diode:BAT54W"

_cnum = [100]


def R(ref, val, a, b, fp=R0603, **kw):
    return P(ref, "Device:R", val, fp, {1: a, 2: b}, **kw)


def C(ref, val, a, b, fp=C0603, **kw):
    return P(ref, "Device:C", val, fp, {1: a, 2: b}, **kw)


def DEC(ic, vcc, gnd="GND", val="100n", **kw):
    """decoupling capacitor at an IC's supply pin"""
    ref = f"C{_cnum[0]}"
    _cnum[0] += 1
    return C(ref, val, vcc, gnd, origin=f"added: decoupling for {ic}", basis="100 nF X7R at the supply pin", **kw)


def TP(ref, net, what, **kw):
    kw.setdefault("origin", "netlist v4" if ref in TESTS_TP else "added: test point")
    return P(ref, "Connector:TestPoint", what, TPFP, {1: net}, basis="safety_board_tests.md scope point" if ref in TESTS_TP else "", **kw)


def STUD(ref, net, what, fp=STUD5, **kw):
    kw.setdefault("origin", "added: board terminal for an off-board part")
    return P(ref, "Connector_Generic:Conn_01x01", what, fp, {1: net}, **kw)


def G1(ref, lib, a, b, y, vcc="+3V3", **kw):
    """single 2-input gate (74LVC1G08/32/86): 1 A, 2 B, 3 GND, 4 Y, 5 VCC"""
    return P(ref, f"74xGxx:{lib}", lib, SOT235, {1: a, 2: b, 3: "GND", 4: y, 5: vcc}, **kw)


def G3(ref, lib, a, b, c, y, **kw):
    """3-input gate (74LVC1G11/332): 1, 3, 6 inputs, 2 GND, 4 Y, 5 VCC"""
    nick = PLIB if lib == "74LVC1G332" else "74xGxx"      # the KiCad 7 symbol has its inputs 2 µm off the 50 mil grid
    return P(ref, f"{nick}:{lib}", lib, SOT236, {1: a, 3: b, 6: c, 2: "GND", 4: y, 5: "+3V3"}, **kw)


def INV(ref, lib, a, y, **kw):
    """74LVC1G04 / 14 / 17: 2 A, 3 GND, 4 Y, 5 VCC"""
    pins = {2: a, 3: "GND", 4: y, 5: "+3V3"}
    if lib in ("74LVC1G17", "74LVC1G14"):
        pins[1] = None
    return P(ref, f"74xGxx:{lib}", lib, SOT235, pins, **kw)


def ONESHOT(ref, a_n, b, clr_n, q, r_ref, c_ref, r_val, c_val, **kw):
    """SN74LVC1G123: 1 An, 2 B, 3 CLRn, 4 GND, 5 Q, 6 Cext, 7 Rext/Cext, 8 VCC; R from Rext/Cext to VCC, C across 6-7"""
    basis = kw.pop("basis", "")
    rn, cn = f"{ref}_RC", f"{ref}_C"
    P(ref, "74xGxx:74LVC1G123", "74LVC1G123", VSSOP8, {1: a_n, 2: b, 3: clr_n, 5: q, 6: cn, 7: rn, 4: "GND", 8: "+3V3"},
      units=[1, 2], basis=basis, **kw)
    sub = dict(origin=kw.get("origin", "netlist v4"), nl=kw.get("nl", ref), variant=kw.get("variant", "all"))
    R(r_ref, r_val, "+3V3", rn, basis=basis, **sub)
    C(c_ref, c_val, cn, rn, fp=C1206 if "u" in c_val else C0603, basis=basis + " (C0G / film)", **sub)
    DEC(ref, "+3V3", variant=sub["variant"])


def DFF(ref, clk, d, clr_n, pre_n, q, qn, **kw):
    P(ref, f"{PLIB}:74LVC1G74", "74LVC1G74", VSSOP8, {1: clk, 2: d, 3: qn, 4: "GND", 5: q, 6: clr_n, 7: pre_n, 8: "+3V3"}, **kw)
    DEC(ref, "+3V3", variant=kw.get("variant", "all"))


def CMP2(ref, a, b, **kw):
    """TLV3202 dual: a/b = (in+, in−, out) or None (unused half: + to GND, − to 3V3, output open)"""
    a = a or ("GND", "+3V3", None)
    b = b or ("GND", "+3V3", None)
    P(ref, f"{PLIB}:TLV3202", "TLV3202", SOIC8, {3: a[0], 2: a[1], 1: a[2], 5: b[0], 6: b[1], 7: b[2], 8: "+3V3", 4: "GND"},
      units=[1, 2, 3], **kw)
    DEC(ref, "+3V3", variant=kw.get("variant", "all"))


def MOS(ref, val, g, d, s, fp=TO220, lib="Device:Q_NMOS_GDS", **kw):
    return P(ref, lib, val, fp, {1: g, 2: d, 3: s} if lib.endswith("GDS") else {1: g, 2: s, 3: d}, **kw)


TESTS_TP = {f"TP{i}" for i in range(1, 14)}

ADDED_GLUE = "added: glue gate for a netlist boolean term (electrical.md gives the term, not the gate)"


def design():
    PARTS.clear()
    NOTES.clear()
    VALUES.clear()
    _cnum[0] = 100
    clamp = el.CLAMP
    v_on, v_off = clamp["v_on"] + clamp["hyst"], clamp["v_on"] - clamp["hyst"]
    ov = ov_divider(v_on, v_off)
    c_ex = el.CMP_FILTER / ov["r_th"]                       # E12 C0G nearest to the 2 µs filter
    c_filt = min((m * 10 ** k for m in (1.0, 1.2, 1.5, 1.8, 2.2, 2.7, 3.3, 3.9, 4.7, 5.6, 6.8, 8.2) for k in range(-12, -8)),
                 key=lambda c: abs(math.log(c / c_ex)))
    pre = preok_network(3.0)
    r_wd = el.WATCHDOG["t_nom"] / (LVC_K * 100e-9)
    r_force = el.T_FORCE / (LVC_K * 1e-6)
    c_on = 4.7e-9
    r_on = e96(el.T_ON_MIN / (LVC_K * c_on))
    frac = 1 - math.exp(-1)
    rt_top, rt_bot, rt_frac = ratio_ref(frac)
    r_disc = e96(el.DISC_WINDOW["t"] / 1e-6 * math.log(1 / (1 - frac)) / math.log(1 / (1 - rt_frac)))
    r_pt = e96(el.T_PRE_TIMEOUT / 1e-6 * math.log(1 / (1 - frac)) / math.log(1 / (1 - rt_frac)))
    D = el.AUX_DEBOUNCE
    r71_ = e96(10e3 * (10.5 / 1.242 - 1))
    thr_ = (2.5 + 16.0 * 0.066) / 2
    r77_ = e96(10e3 * thr_ / (VDD - thr_))
    r101_ = e96(0.050 / 1e-6 * math.log(1 / (1 - frac)) / math.log(1 / (1 - rt_frac)))
    tk = math.log(1 / (1 - rt_frac))
    VALUES.extend([
        ("clamp on / off (U4, U23)", f"{v_on:.2f} / {v_off:.2f} V (CLAMP v_on ± hyst)", f"{ov['on']:.3f} / {ov['off']:.3f} V (R44-R46 E96 0.1 %, VREF 2.495 V)"),
        ("comparator filter", f"{el.CMP_FILTER * 1e6:.1f} µs (CMP_FILTER)", f"{ov['r_th'] * c_filt * 1e6:.2f} µs (Rth {ov['r_th'] / 1e3:.2f}k × {fc(c_filt)})"),
        ("PRE_OK", "VBUS ≥ VIN − 3 V (table)", f"VBUS ≥ VIN − {pre['offset']:.2f} V (k+ {pre['k_plus']:.4f} / k− {pre['k_minus']:.4f})"),
        ("watchdog U5 / U22", f"{el.WATCHDOG['t_nom'] * 1e3:.0f} ms (bringup.yaml)", f"{r_wd * 100e-9 * 1e3:.1f} ms (R4 {fr(r_wd)} × 100 nF, K = 1)"),
        ("min on-time U16 / U24", f"{el.T_ON_MIN * 1e6:.0f} µs (T_ON_MIN)", f"{r_on * c_on * 1e6:.1f} µs ({fr(r_on)} × {fc(c_on)})"),
        ("FORCE U6 / FORCE_B U25", f"{el.T_FORCE * 1e3:.0f} ms (T_FORCE)", f"{r_force * 1e-6 * 1e3:.0f} ms (100k × 1 µF)"),
        ("pre-charge timeout U11", f"{el.T_PRE_TIMEOUT:.1f} s (T_PRE_TIMEOUT)", f"{r_pt * 1e-6 * tk:.2f} s ({fr(r_pt)} × 1 µF to {rt_frac:.3f}·VDD)"),
        ("discrepancy window U17", f"{el.DISC_WINDOW['t'] * 1e3:.0f} ms ± {el.DISC_WINDOW['tol']:.0%}", f"{r_disc * 1e-6 * tk * 1e3:.1f} ms ({fr(r_disc)} × 1 µF)"),
        ("DISC_B window U26", "50 ms (table)", f"{r101_ * 1e-6 * tk * 1e3:.1f} ms"),
        ("12 V UVLO U18", "10.5 V (table)", f"{1.242 * (1 + r71_ / 10e3):.2f} V ({fr(r71_)} / 10k, REF 1.242 V)"),
        ("branch overload U19", "16 A, 5 s (table)", f"{(2 * VDD * r77_ / (10e3 + r77_) - 2.5) / 0.066:.1f} A, 5.0 s (66 mV/A ESTIMATE)"),
        ("AUX debounce R_c / R_d / C_db", f"{fr(D['r_c'])} / {fr(D['r_d'])} / {fc(D['c'])} (AUX_DEBOUNCE)", f"R20 {fr(D['r_c'])} / R21 {fr(D['r_d'])} / C20 {fc(D['c'])}"),
        ("pre-charge R, bleeders, bulk C, clamp R", f"{el.R_PRE} Ω, {fr(el.R_BLEED)}, {el.C_BULK * 1e3:.1f} mF, {clamp['r_each']} Ω",
         f"R5 {el.R_PRE}, R6/R8 {fr(el.R_BLEED)} 3 W, C1 {el.C_BULK * 1e3:.1f} mF, R1-R3/R7 {clamp['r_each']}"),
    ])
    # drift guards on the table text the drawing relies on (values with no electrical.py constant)
    need("F1", "60 A MIDI", "PSU+", "F1_OUT")
    need("U1, Q6", "LM74700", "IRFB7530", "F1_OUT → VIN")
    need("K1", "ALEV100", "VIN ↔ VBUS", "COIL+ / COIL−", "AUX_A / AUX_B")
    need("D1, Z1", "1N4007", "1N5359B", "in series across the K1 coil")
    need("Q4", "100 V", "IRL540N", "COIL− → GND", "gate U8_OUT")
    need("U8", "SN74LVC1G11", "SN74LVC1G32 OR (PRE_OK ∨ AUX_DB)")
    need("S1, S2", "S1a/S2a: +24V_L → COIL+ (series)", "S1b/S2b: 3V3 → U10 / U13")
    need("SB1", "3V3 → U10 CLK")
    need("U10", "SN74LVC1G74")
    need("U28", "D = 3V3, CLK = AUX_DB (rising), CLR̄ = LATCH → RUN")
    need("U29, R_c, R_d, D_db, C_db", "74LVC1G17", "BAT54")
    need("R9, C9", "100 kΩ + 1 µF", "3V3 → CLR̄")
    need("K2, Q5", "G5LE-1", "IRLML6344", "VIN → K2_C → R3 → VBUS", "gate = K2_DRIVE")
    need("R3", "RH-50", "K2_C → VBUS")
    need("U11", "1 s", "PRE_TIMEOUT")
    need("U4", "TLV3202", "TL431B", "0.1 %", "PRE_OK (VBUS ≥ VIN − 3 V)")
    need("U16", "50 µs", "B = OV (rising)", "ORed into U7")
    need("U17", "SN74LVC1G86", "XOR(Q4 gate, AUX_DB) → DISC")
    need("U18", "TLV3012", "10.5 V", "U2 output → UV12")
    need("U19", "ACS724-30A", "5 s", "16 A", "VBUS → F3 branch → OC")
    need("U6", "R 100 kΩ, C 1 µF", "Ā = AUX_DB", "B = 3V3", "CLR̄ = ¬K2_DRIVE")
    need("U15", "SN74LVC1G04", "K2_DRIVE → U6 CLR̄")
    need("U7, U9", "SN74LVC1G32", "UCC27524", "OV ∨ FORCE")
    need("Q1–Q3 / R1–R3", "IRFB7530", "NH-50 0.56 Ω", "VBUS → R → Q → GND")
    need("D4–D6", "VS-40CPQ060", "anode at Q drain, cathode at VBUS")
    need("TH1", "85 °C NC")
    need("R6", "1 kΩ 3 W", "VBUS → GND")
    need("C1, C2, D3", "4.7 mF 35 V", "10 µF film", "SMDJ33A")
    need("U5", "R4 250 kΩ", "C3 100 nF C0G", "Ā = GND", "Q = WD_OK")
    need("U2, U3", "24 → 12 V 1 A buck", "MCP1703A-3302", "F2 (1 A) from VIN")
    need("U12, U13", "PC817", "GPIO27", "GPIO22")
    need("F3", "20 A ATO")
    need("U20", "G9SA-321-T075")
    need("U27", "AQV252G", "LED driven by LATCH")
    need("Q9, U21", "LM5069-1", "1 mΩ sense", "VIN → VBUS_B")
    need("U22", "25 ms", "GPIO23", "Q = WD_B")
    need("Q10, R7, D7, U23, U24, U22b", "UCC27517", "VBUS_B → R7 → Q10 → GND")
    need("U25", "100 ms", "Ā = U21 EN (falling)")
    need("U26", "50 ms", "DISC_B (GPIO24)")
    need("R8", "1 kΩ 3 W", "VBUS_B → GND")
    need("Q11", "flyback diode", "100 kΩ pull-down")
    need_art("F2 1A", "F 20A", "U21 EN ← +24V_L ─ U20 delayed contact 1 ─ U20 delayed contact 2",
             "ch1 = S1c ─ S2c ─ U27 photoMOS (LATCH), ch2 = S1d ─ S2d; reset = SB1b", "GPIO17 heartbeat ─ U5a 74LVC1G17",
             "AUX_RAW (TP8)", "AUX_DB (TP12)", "RUN (TP13)")
    if el.R_PRE != 3.3 or el.BENCH_FUSE != 20 or el.MAIN_FUSE["rating"] != 60:
        print("note: electrical.py fuse / pre-charge constants changed; the drawing follows them")

    # ============================================================ sheet 1: power path, contactor, pre-charge, logic supply
    sheet("power")
    group("Supply input, F1, ideal diode U1/Q6 (blocks regen into the PSU)")
    P("J1", "Connector_Generic:Conn_01x02", "Supply / pack output", "", {1: "PSU+", 2: "PGND"}, loc="chassis",
      origin="added: the supply / pack cable (PSU+ to F1, PSU- to stud J3)", basis="bench SE-1000-24 or the pack via XT90")
    P("F1", "Device:Fuse", f"{el.MAIN_FUSE['rating']}A MIDI slow-blow", "", {1: "PSU+", 2: "F1_OUT"}, loc="chassis", nl="F1",
      basis="electrical.py MAIN_FUSE")
    STUD("J2", "F1_OUT", "M5 stud F1_OUT (from F1)")
    STUD("J3", "PGND", "M5 stud PSU- / pack-")
    P("U1", "Power_Management:LM74700", "LM74700-Q1", "Package_TO_SOT_SMD:SOT-23-6",
      {1: "U1_VCAP", 2: "GND", 3: "F1_OUT", 4: "VIN", 5: "Q6_G", 6: "F1_OUT"}, nl="U1, Q6", basis="EN tied to ANODE (always on)")
    C("C10", "100n 50V", "U1_VCAP", "F1_OUT", origin="added: LM74700 VCAP charge-pump capacitor (datasheet 0.1 µF)", nl="U1, Q6")
    MOS("Q6", "IRFB7530", "Q6_G", "VIN", "F1_OUT", nl="U1, Q6", basis="source at the supply side: body diode F1_OUT → VIN")
    STUD("J4", "VIN", "M5 stud VIN (to K1 A1)")
    P("#FLG01", "power:PWR_FLAG", "PWR_FLAG", "", {1: "F1_OUT"}, origin="ERC flag")
    P("#FLG02", "power:PWR_FLAG", "PWR_FLAG", "", {1: "VIN"}, origin="ERC flag")
    P("#FLG03", "power:PWR_FLAG", "PWR_FLAG", "", {1: "PGND"}, origin="ERC flag")
    P("#FLG04", "power:PWR_FLAG", "PWR_FLAG", "", {1: "GND"}, origin="ERC flag")

    group("K1 main contactor (chassis) and coil driver Q4 (12 V gate drive from U9 channel B)")
    P("K1", f"{PLIB}:ALEV100", "ALEV100 24V coil", "", {"A1": "VIN", "A2": "VBUS", "X1": "COIL+", "X2": "COIL-", "3": "AUX_A", "4": "AUX_B"},
      loc="chassis", nl="K1", basis="electrical.py CONTACTOR")
    STUD("J5", "VBUS", "M5 stud VBUS (from K1 A2)")
    P("NT1", "Device:NetTie_2", "VBUS sense tie", "NetTie:NetTie-2_SMD_Pad0.5mm", {1: "VBUS", 2: "VBUS_SNS"},
      origin="added: Kelvin sense point for the comparator dividers (checklist: divider sense at the VBUS terminal)")
    P("D1", "Device:D", "1N4007", "Diode_THT:D_DO-41_SOD81_P10.16mm_Horizontal", {1: "COIL+", 2: "K1_SNUB"}, nl="D1, Z1")
    P("Z1", "Device:D_Zener", "1N5359B 24V 5W", "Diode_THT:D_DO-201AE_P15.24mm_Horizontal", {1: "COIL-", 2: "K1_SNUB"}, nl="D1, Z1",
      basis="drain clamps at 24 V + Vz + Vf ≈ 50 V (safety_board_sim.md: 52 V)")
    MOS("Q4", "IRL540N 100V", "Q4_G", "COIL-", "PGND", nl="Q4", basis="design change 1: driven from U9 OUTB at 12 V (not specified at 3.3 V)")
    R("R15", "10", "Q4_DRV", "Q4_G", origin="added: Q4 gate resistor", nl="Q4")
    R("R16", "10k", "Q4_G", "PGND", origin="added: Q4 gate pull-down (checklist: dead driver = off)", nl="Q4")
    TP("TP6", "Q4_G", "TP6 Q4 gate")
    TP("TP7", "COIL-", "TP7 Q4 drain")
    TP("TP29", "PGND", "TP29 PGND (Q4)", origin="added: scope ground next to TP6/TP7")

    group("K2 pre-charge relay + R3 (RH-50, chassis); +24V_L feed")
    P("K2", "Relay:G5LE-1", "G5LE-1 24VDC", "Relay_THT:Relay_SPDT_Omron-G5LE-1", {1: "VIN", 2: "K2_COIL-", 3: None, 4: "K2_C", 5: "COIL+"},
      nl="K2, Q5", basis="coil from the coil chain node COIL+ (block diagram: '+24V_L ─ S1a/S2a (same chain) ─ K2 coil'); NO = pin 4")
    P("D9", "Device:D", "1N4148W", "Diode_SMD:D_SOD-123", {1: "COIL+", 2: "K2_COIL-"},
      origin="added: K2 coil flyback diode (checklist: coil suppression on K2 too; not in the netlist)", nl="K2, Q5")
    MOS("Q5", "IRLML6344", "Q5_G", "K2_COIL-", "PGND", fp=SOT23, lib="Device:Q_NMOS_GSD", nl="K2, Q5",
        basis="netlist part; 30 V (design change 1 asks >= 60 V: issue N5)")
    R("R17", "100", "K2_DRIVE", "Q5_G", origin="added: Q5 gate series resistor", nl="K2, Q5")
    R("R18", "100k", "Q5_G", "PGND", origin="added: Q5 gate pull-down", nl="K2, Q5")
    P("J6", "Connector_Generic:Conn_01x02", "Pre-charge resistor leads", "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2-5.08_1x02_P5.08mm_Horizontal",
      {1: "K2_C", 2: "VBUS"}, origin="added: board terminal for the off-board RH-50")
    P("R5", "Device:R", f"{el.R_PRE} RH-50", "", {1: "K2_C", 2: "VBUS"}, loc="chassis", nl="R3",
      origin="netlist v4 (as 'R3': duplicate ref with the clamp NH-50 R3, issue N1)", basis="electrical.py R_PRE")
    P("F5", "Device:Fuse", "2A slow 1206", "Fuse:Fuse_1206_3216Metric", {1: "VIN", 2: "+24V_L"},
      origin="added: +24V_L has no source in the netlist (issue N2); fused from VIN, separate from F2 ('nothing else may hang on it')",
      basis="K1 coil + K2 coil + U20 + aux opto ≈ 0.4 A ESTIMATE")

    group("Bulk capacitor C1 (star point PGND/GND), arm branch: U19 current sensor, F3")
    P("C1", "Device:C_Polarized", f"{el.C_BULK * 1e3:.1f}mF 35V", "Capacitor_THT:CP_Radial_D30.0mm_P10.00mm_SnapIn",
      {1: "VBUS", 2: "PGND"}, nl="C1, C2, D3", basis="electrical.py C_BULK")
    P("NT2", "Device:NetTie_2", "Star point at C1-", "NetTie:NetTie-2_SMD_Pad2.0mm", {1: "PGND", 2: "GND"},
      origin="added: the only joint between power return and logic ground (checklist: star point at C1)")
    P("U19", "Sensor_Current:ACS724xLCTR-30AB", "ACS724LLCTR-30AB", SOIC8,
      {1: "VBUS", 2: "VBUS", 3: "ARM_F", 4: "ARM_F", 5: "GND", 6: "U19_FILT", 7: "OC_V", 8: "+5V"}, nl="U19",
      basis="66 mV/A at 5 V (ESTIMATE until the datasheet is checked)")
    C("C53", "1n", "U19_FILT", "GND", origin="added: U19 pin 6 (FILTER per the KiCad symbol; issue R6)", nl="U19")
    DEC("U19", "+5V")
    P("F3", "Device:Fuse", f"{el.BENCH_FUSE}A ATO", ATO, {1: "ARM_F", 2: "ARM_LEAD"}, nl="F3", basis="electrical.py BENCH_FUSE")
    STUD("J11", "ARM_LEAD", "M4 stud arm lead +", fp=STUD4)
    STUD("J12", "PGND", "M4 stud arm lead -", fp=STUD4)
    TP("TP1", "VBUS", "TP1 VBUS at the board")
    P("TP2", "Connector:TestPoint", "TP2 VBUS at the dummy drive bank", "", {1: "ARM_LEAD"}, loc="chassis",
      basis="safety_board_tests.md: at the end of the arm lead (off-board probe point)")

    group("Logic supply: F2 → U2 24→12 V → U3 3.3 V (watchdog/logic only), U32 5 V for U19")
    P("F2", "Device:Fuse", "1A slow 1206", "Fuse:Fuse_1206_3216Metric", {1: "VIN", 2: "F2_OUT"}, nl="U2, U3")
    P("#FLG05", "power:PWR_FLAG", "PWR_FLAG", "", {1: "F2_OUT"}, origin="ERC flag")
    P("U2", "Regulator_Switching:R-78B12-2.0", "R-78B12-2.0", "Converter_DCDC:Converter_DCDC_RECOM_R-78B-2.0_THT",
      {1: "F2_OUT", 2: "GND", 3: "+12V"}, nl="U2, U3", basis="netlist '24 → 12 V 1 A buck'; R-78B: 32 V max input (R-78E is 28 V, too close to the 27.5 V drive limit)")
    C("C12", "10u 50V", "F2_OUT", "GND", fp=C1210, origin="added: U2 input capacitor", nl="U2, U3")
    C("C13", "10u 25V", "+12V", "GND", fp=C1210, origin="added: U2 output capacitor", nl="U2, U3")
    P("U3", "Regulator_Linear:MCP1703Ax-330xxDB", "MCP1703A-3302", "Package_TO_SOT_SMD:SOT-223-3_TabPin2",
      {1: "+12V", 2: "GND", 3: "+3V3"}, nl="U2, U3")
    C("C14", "1u", "+12V", "GND", origin="added: MCP1703 input 1 µF (datasheet)", nl="U2, U3")
    C("C15", "1u", "+3V3", "GND", origin="added: MCP1703 output 1 µF (datasheet)", nl="U2, U3")
    P("U32", "Regulator_Linear:MCP1703Ax-500xxDB", "MCP1703A-5002", "Package_TO_SOT_SMD:SOT-223-3_TabPin2",
      {1: "+12V", 2: "GND", 3: "+5V"}, origin="added: the ACS724 is a 5 V part and the netlist has no 5 V rail (issue N3)")
    C("C16", "1u", "+12V", "GND", origin="added: MCP1703 input", nl="U19")
    C("C17", "1u", "+5V", "GND", origin="added: MCP1703 output", nl="U19")
    TP("TP14", "+12V", "TP14 +12V (T0)")
    TP("TP15", "+3V3", "TP15 +3V3 (T0)")
    TP("TP26", "GND", "TP26 GND")
    NOTES["power"] = [
        "Power path: PSU+ -> F1 (60 A MIDI, chassis) -> J2 -> Q6/U1 ideal diode -> VIN -> J4 -> K1 (chassis) -> J5 -> VBUS.",
        "Pre-charge: VIN -> K2 (NO, pin 4) -> J6 -> R3/RH-50 3.3 ohm (drawn R5, chassis) -> VBUS. K2 coil and K1 coil both hang on COIL+, after S1a-S2a.",
        "K1 coil: +24V_L -> S1a -> S2a -> COIL+ -> coil -> COIL- -> Q4 (100 V) -> PGND; D1 + Z1 across the coil (clamp ~50 V).",
        "Q4 is driven at 12 V from U9 OUTB (design change 1); its logic command is U8_OUT = LATCH & WD_OK & (PRE_OK | AUX_DB).",
        "PGND (power return) and GND (logic) meet only at NT2, at C1-. VBUS_SNS is Kelvin-tied to VBUS at NT1 (J5).",
    ]

    # ============================================================ sheet 2: regen clamp
    sheet("clamp")
    group("Clamp legs: R1-R3 NH-50 0.56 ohm (chassis plate) ∥ D4-D6, Q1-Q3 IRFB7530")
    STUD("J10", "VBUS", "M5 stud VBUS (NH-50 common leads)")
    for i in (1, 2, 3):
        STUD(f"J{6 + i}", f"DRN{i}", f"M4 stud DRN{i} (R{i} lead)", fp=STUD4)
        P(f"R{i}", "Device:R", f"{clamp['r_each']} NH-50", "", {1: "VBUS", 2: f"DRN{i}"}, loc="chassis", nl="Q1–Q3 / R1–R3",
          basis="electrical.py CLAMP r_each; Vishay NH-50 on the Al plate, 10 AWG leads")
        P(f"D{3 + i}", f"{PLIB}:VS-40CPQ060", "VS-40CPQ060-N3", TO247, {1: f"DRN{i}", 3: f"DRN{i}", 2: "VBUS"}, nl="D4–D6")
        MOS(f"Q{i}", "IRFB7530", f"G{i}", f"DRN{i}", "PGND", nl="Q1–Q3 / R1–R3")
        R(f"R{10 + i}", "10 1206", "CLAMP_DRV", f"G{i}", fp=R1206, nl="R_G", basis="≈ 10–22 ohm, set on the scope for ≈ 0.3 µs edges")
        R(f"R{39 + i}", "10k", f"G{i}", "PGND", origin="added: gate pull-down (checklist)", nl="Q1–Q3 / R1–R3")
    C("C2", "10u 63V film", "VBUS", "PGND", fp=FILM10U, nl="C1, C2, D3", basis="at Q1-Q3: closes the 30 nH MOSFET-diode loop")
    P("D3", "Device:D_Zener", "SMDJ33A TVS", "Diode_SMD:D_SMC", {1: "VBUS", 2: "PGND"}, nl="C1, C2, D3", basis="edge snubber only (unidirectional, cathode at VBUS)")
    R("R6", f"{fr(el.R_BLEED)} 3W", "VBUS", "PGND", fp=R3W, nl="R6", basis="electrical.py R_BLEED")
    TP("TP3", "DRN1", "TP3 Q1 drain")
    TP("TP4", "G1", "TP4 Q1 gate")
    TP("TP27", "PGND", "TP27 PGND at Q1 source", origin="added: scope ground spring for TP3/TP4")

    group("Gate drive: U7 OR3 (OV | U16 | FORCE) → U9 UCC27524 (A: clamp, B: K1 coil Q4)")
    G3("U7", "74LVC1G332", "OV", "U16_Q", "FORCE", "CLAMP_CMD", nl="U7, U9",
       basis="3 terms (OV, FORCE and U16's on-time): the netlist's 2-input SN74LVC1G32 cannot take them (issue N4)")
    DEC("U7", "+3V3")
    P("U9", f"{PLIB}:UCC27524", "UCC27524", SOIC8, {1: "+12V", 2: "CLAMP_CMD", 3: "PGND", 4: "U8_OUT", 5: "Q4_DRV", 6: "+12V", 7: "CLAMP_DRV", 8: "+12V"},
      nl="U7, U9", basis="ENA/ENB tied to VDD; channel B drives Q4 (design change 1)")
    C("C18", "1u 25V", "+12V", "PGND", fp=C1206, origin="added: UCC27524 VDD bulk", nl="U7, U9")
    DEC("U9", "+12V", "PGND")
    ONESHOT("U16", "GND", "OV", "+3V3", "U16_Q", "R52", "C46", fr(r_on), fc(c_on) + " C0G", nl="U16",
            basis=f"t = R·C = {r_on * c_on * 1e6:.1f} µs (electrical.py T_ON_MIN {el.T_ON_MIN * 1e6:.0f} µs)")

    group("Clamp comparator U4 (A: OV on/off, B: PRE_OK) + TL431B reference")
    CMP2("U4", ("OV_P", "VREF", "OV"), ("PRE_P", "PRE_N", "PRE_OK"), nl="U4")
    P("U33", "Reference_Voltage:TL431DBZ", "TL431B", SOT23, {1: "VREF", 2: "VREF", 3: "GND"}, nl="U4",
      basis="TL431B as a 2.495 V shunt (K = REF); also the reference of U23")
    R("R43", "680", "+3V3", "VREF", origin="added: TL431 bias (≈ 1.2 mA ≥ 1 mA min)", nl="U4")
    C("C44", "1n C0G", "VREF", "GND", origin="added: TL431 decoupling (checklist); below the TL431 stability boundary, reviewer re-check", nl="U4")
    R("R44", fr(ov["r_top"]) + " 0.1%", "VBUS_SNS", "OV_P", nl="U4", basis=f"OV divider: on {ov['on']:.2f} V / off {ov['off']:.2f} V "
      f"(electrical.py CLAMP v_on ± hyst = {v_on:.2f} / {v_off:.2f} V)")
    R("R45", fr(ov["r_bot"]) + " 0.1%", "OV_P", "GND", nl="U4", basis="OV divider bottom")
    R("R46", fr(ov["r_hys"]) + " 0.1%", "OV", "OV_P", nl="U4", basis="positive feedback: 0.30 V hysteresis")
    C("C45", fc(c_filt) + " C0G", "OV_P", "GND", nl="U4",
      basis=f"2 µs filter (design change 0): Rth {ov['r_th'] / 1e3:.2f}k × {fc(c_filt)} = {ov['r_th'] * c_filt * 1e6:.2f} µs")
    R("R47", fr(pre["ra"]) + " 0.1%", "VBUS_SNS", "PRE_P", nl="U4", basis=f"PRE_OK + node: {pre['k_plus']:.4f}·VBUS + offset")
    R("R48", fr(pre["rb"]) + " 0.1%", "VREF", "PRE_P", nl="U4", basis=f"offset from VREF: PRE_OK at VBUS ≥ VIN − {pre['offset']:.2f} V")
    R("R49", fr(pre["rc"]) + " 0.1%", "PRE_P", "GND", nl="U4", basis="PRE_OK + node to GND")
    R("R50", fr(pre["rd"]) + " 0.1%", "VIN", "PRE_N", nl="U4", basis=f"PRE_OK − node: {pre['k_minus']:.4f}·VIN")
    R("R51", fr(pre["re"]) + " 0.1%", "PRE_N", "GND", nl="U4", basis="PRE_OK − node to GND")
    TP("TP22", "PRE_OK", "TP22 PRE_OK")
    TP("TP23", "OV", "TP23 OV")
    NOTES["clamp"] = [
        f"Clamp window from electrical.py CLAMP: on {v_on:.2f} V, off {v_off:.2f} V; drawn values give on {ov['on']:.3f} V / off {ov['off']:.3f} V (E96 0.1 %).",
        "Gate = OV | U16 (50 us minimum on-time, triggered by OV rising) | FORCE, through U7 -> U9 OUTA -> R11-R13 -> Q1-Q3.",
        "Freewheel: D4-D6 anodes (pins 1 + 3) at each drain, cathode (pin 2 + tab) at VBUS: across each NH-50 (finding 1).",
        "Loop to minimise on the board: C2 -> J10 -> (NH-50 leads, off-board) -> J7-J9 -> D4-D6 / Q1-Q3 -> PGND -> C2.",
    ]

    # ============================================================ sheet 3: logic
    sheet("logic")
    group("Arm watchdog: GPIO17 → U5a (U30) Schmitt → U5 '123, 25 ms → WD_OK")
    R("R55", "100", "GPIO17", "HB_IN", origin="added: series resistor on the Pi line", nl="U5")
    R("R56", "100k", "HB_IN", "GND", origin="added: pull-down (Pi unplugged = no edges)", nl="U5")
    INV("U30", "74LVC1G17", "HB_IN", "HB_BUF", nl="U5a", origin="netlist v4 (as 'U5a')")
    DEC("U30", "+3V3")
    ONESHOT("U5", "GND", "HB_BUF", "+3V3", "WD_OK", "R4", "C3", fr(r_wd) + " 1%", "100n C0G", nl="U5",
            basis=f"t = R4·C3 = {r_wd * 100e-9 * 1e3:.0f} ms (bringup.yaml watchdog_s)")
    TP("TP5", "WD_OK", "TP5 WD_OK")

    group("E-stop channel b (S1b-S2b in series = S1b ∧ S2b) and U13 → GPIO22 ESTOP_SENSE")
    R("R57", "10k", "ESTOP_CH2", "GND", origin="added: pull-down (open loop = stop)", nl="S1, S2")
    C("C48", "10n", "ESTOP_CH2", "GND", origin="added: cable noise filter", nl="S1, S2")
    R("R58", "470", "ESTOP_CH2", "ESTOP_LED", origin="added: U13 LED resistor (≈ 4 mA)", nl="U12, U13")
    P("U13", "Isolator:PC817", "PC817", "Package_DIP:DIP-4_W7.62mm", {1: "ESTOP_LED", 2: "GND", 3: "GND", 4: "GPIO22"}, nl="U12, U13",
      basis="open collector to GPIO22 (Pi pull-up)")
    R("R60", "10k", "+3V3", "AUX_RAW_N", origin="added: U12 collector pull-up", nl="U12, U13")

    group("K1 aux: AUX_A ← +24V_L, AUX_B → U12 → U58 → AUX_RAW (TP8) → GPIO27; debounce U29 → AUX_DB (TP12)")
    R("R59", "4.7k", "+24V_L", "AUX_A", origin="added: aux loop current ≈ 5 mA (+24V_L → aux → U12 LED)", nl="U12, U13")
    P("U12", "Isolator:PC817", "PC817", "Package_DIP:DIP-4_W7.62mm", {1: "AUX_B", 2: "GND", 3: "GND", 4: "AUX_RAW_N"}, nl="U12, U13")
    INV("U58", "74LVC1G14", "AUX_RAW_N", "AUX_RAW", origin="added: stiff push-pull AUX_RAW for the debounce RC (its model assumes a stiff source; "
        "an opto emitter with a pull-down would add ~10k to R_d); the PC817 collector output is inverted back", nl="U12, U13")
    DEC("U58", "+3V3")
    R("R61", "100", "AUX_RAW", "GPIO27", origin="added: series resistor to the Pi (AUX_OK)", nl="U12, U13")
    TP("TP8", "AUX_RAW", "TP8 AUX_RAW")
    R("R20", fr(D["r_c"]) + " 1%", "AUX_RAW", "AUX_C", nl="R_c", origin="netlist v4 (as 'R_c')", basis="electrical.py AUX_DEBOUNCE r_c")
    P("D20", BAT54, "BAT54W", "Package_TO_SOT_SMD:SOT-323_SC-70", {1: "AUX_C", 3: "AUX_RD", 2: None}, nl="D_db", origin="netlist v4 (as 'D_db')",
      basis="anode at C_db: discharges it when AUX_RAW falls")
    R("R21", fr(D["r_d"]) + " 1%", "AUX_RD", "AUX_RAW", nl="R_d", origin="netlist v4 (as 'R_d')", basis="electrical.py AUX_DEBOUNCE r_d")
    C("C20", fc(D["c"]) + " C0G", "AUX_C", "GND", nl="C_db", origin="netlist v4 (as 'C_db')", basis="electrical.py AUX_DEBOUNCE c")
    INV("U29", "74LVC1G17", "AUX_C", "AUX_DB", nl="U29, R_c, R_d, D_db, C_db")
    DEC("U29", "+3V3")
    TP("TP12", "AUX_DB", "TP12 AUX_DB")
    INV("U60", "74LVC1G04", "AUX_DB", "AUX_DB_N", origin=ADDED_GLUE + ": ¬AUX_DB", nl="U10")
    DEC("U60", "+3V3")

    group("STOP / ARM terms: RUN_OK = ¬STOP = S1b∧S2b∧WD_OK∧TH1∧WD_B; ARM_OK (U10 D) = RUN_OK ∧ ¬AUX_DB")
    G3("U40", "74LVC1G11", "ESTOP_CH2", "WD_OK", "TH1_OK", "OK3", origin=ADDED_GLUE, nl="U10")
    DEC("U40", "+3V3")
    G1("U41", "74LVC1G08", "OK3", "WD_B", "RUN_OK", origin=ADDED_GLUE, nl="U10")
    DEC("U41", "+3V3")
    G1("U61", "74LVC1G08", "RUN_OK", "AUX_DB_N", "ARM_OK", origin=ADDED_GLUE, nl="U10")
    DEC("U61", "+3V3")
    P("JP1", "Jumper:SolderJumper_2_Open", "WD_B=1 (M1 only)", "Jumper:SolderJumper-2_P1.3mm_Open_RoundedPad1.0x1.5mm", {1: "+3V3", 2: "WD_B"},
      variant="M1", origin="added: on the M1 bench (no base channel, U22 not fitted) WD_B is tied high; open on the robot", nl="U22")

    group("LATCH clear: CLR̄ = ¬(STOP ∨ FORCE ∨ PRE_TIMEOUT ∨ DISC ∨ UV12 ∨ OC), POR = R9·C9 (wired-AND through D21)")
    G3("U43", "74LVC1G332", "FORCE", "PRE_TIMEOUT", "DISC", "FAULT_A", origin=ADDED_GLUE, nl="U10")
    DEC("U43", "+3V3")
    G3("U44", "74LVC1G332", "FAULT_A", "UV12", "OC", "FAULT", origin=ADDED_GLUE, nl="U10")
    DEC("U44", "+3V3")
    INV("U62", "74LVC1G04", "FAULT", "FAULT_N", origin=ADDED_GLUE, nl="U10")
    DEC("U62", "+3V3")
    G1("U63", "74LVC1G08", "RUN_OK", "FAULT_N", "CLR_CMD", origin=ADDED_GLUE, nl="U10")
    DEC("U63", "+3V3")
    P("D21", BAT54, "BAT54W", "Package_TO_SOT_SMD:SOT-323_SC-70", {1: "LATCH_CLRn", 3: "CLR_CMD", 2: None},
      origin="added: wired-AND of the clear command with the R9·C9 POR node", nl="R9, C9")
    R("R9", "100k", "+3V3", "LATCH_CLRn", nl="R9, C9", basis="POR τ = 0.1 s (electrical.md)")
    C("C9", "1u", "LATCH_CLRn", "GND", fp=C1206, nl="R9, C9", basis="POR τ = 0.1 s")

    group("ARM button debounce, U10 LATCH (TP10), U28 RUN (TP13)")
    R("R63", "100k", "ARM_RAW", "GND", origin="added: SB1 pull-down", nl="SB1")
    R("R62", "10k", "ARM_RAW", "ARM_RC", origin="added: SB1 debounce (checklist: 'SB1 is the clock, debounced'; not in the netlist)", nl="SB1")
    C("C49", "100n", "ARM_RC", "GND", origin="added: SB1 debounce τ 1 ms", nl="SB1")
    INV("U59", "74LVC1G17", "ARM_RC", "ARM_CLK", origin="added: SB1 debounce Schmitt", nl="SB1")
    DEC("U59", "+3V3")
    DFF("U10", "ARM_CLK", "ARM_OK", "LATCH_CLRn", "+3V3", "LATCH", None, nl="U10")
    TP("TP10", "LATCH", "TP10 LATCH")
    DFF("U28", "AUX_DB", "+3V3", "LATCH", "+3V3", "RUN", "RUN_N", nl="U28")
    TP("TP13", "RUN", "TP13 RUN")

    group("K2_DRIVE = LATCH ∧ ¬RUN ∧ ¬PRE_TIMEOUT; U11 1 s pre-charge timer (TP11 K2_DRIVE)")
    G1("U47", "74LVC1G08", "LATCH", "RUN_N", "K2X", origin=ADDED_GLUE + ": LATCH ∧ ¬RUN (also starts U11)", nl="U11")
    DEC("U47", "+3V3")
    G1("U46", "74LVC1G08", "K2X", "PRE_TO_N", "K2_DRIVE", origin=ADDED_GLUE, nl="K2, Q5")
    DEC("U46", "+3V3")
    TP("TP11", "K2_DRIVE", "TP11 K2_DRIVE")
    R("R64", fr(r_pt), "K2X", "PT_RC", nl="U11", basis=f"τ = {r_pt * 1e-6:.2f} s to {rt_frac:.3f}·3.3 V (electrical.py T_PRE_TIMEOUT {el.T_PRE_TIMEOUT:.1f} s)")
    P("D22", BAT54, "BAT54W", "Package_TO_SOT_SMD:SOT-323_SC-70", {1: "PT_RC", 3: "K2X", 2: None}, origin="added: fast reset of the timer RC", nl="U11")
    C("C50", "1u film", "PT_RC", "GND", fp=C1206, nl="U11")
    R("R65", fr(rt_top), "+3V3", "PT_REF", nl="U11")
    R("R66", fr(rt_bot), "PT_REF", "GND", nl="U11")
    CMP2("U11", ("PT_RC", "PT_REF", "PRE_TIMEOUT"), ("PT_REF", "PT_RC", "PRE_TO_N"), nl="U11",
         basis="RC + comparator; B is the complement (K2_DRIVE term)")
    TP("TP24", "PRE_TIMEOUT", "TP24 PRE_TIMEOUT")

    group("FORCE one-shot U6: Ā = AUX_DB ∧ ¬DISC (falling = K1 opened or DISC), B = 3V3, CLR̄ = ¬K2_DRIVE (U15) (TP9)")
    INV("U15", "74LVC1G04", "K2_DRIVE", "U6_CLRn", nl="U15")
    DEC("U15", "+3V3")
    G1("U52", "74LVC1G08", "AUX_DB", "DISC_N", "U6_TRIG", origin=ADDED_GLUE + ": DISC → U6 trigger (netlist: 'DISC → U6 trigger')", nl="U17")
    DEC("U52", "+3V3")
    ONESHOT("U6", "U6_TRIG", "+3V3", "U6_CLRn", "FORCE", "R67", "C51", fr(r_force), "1u", nl="U6",
            basis=f"t = R·C = {r_force * 1e-6 * 1e3:.0f} ms (electrical.py T_FORCE)")
    TP("TP9", "FORCE", "TP9 FORCE")

    group("Coil command U8 = LATCH ∧ WD_OK ∧ (PRE_OK ∨ AUX_DB); discrepancy U17 (40 ms) → DISC")
    G1("U57", "74LVC1G32", "PRE_OK", "AUX_DB", "HOLD", nl="U8", origin="netlist v4 (U8's OR)")
    DEC("U57", "+3V3")
    G3("U8", "74LVC1G11", "LATCH", "WD_OK", "HOLD", "U8_OUT", nl="U8")
    DEC("U8", "+3V3")
    G1("U17", "74LVC1G86", "U8_OUT", "AUX_DB", "DISC_X", nl="U17", basis="XOR(coil command, AUX_DB); the logic-level command stands for 'Q4 gate' (12 V)")
    DEC("U17", "+3V3")
    R("R68", fr(r_disc), "DISC_X", "DISC_RC", nl="U17", basis=f"window {r_disc * 1e-6 * math.log(1 / (1 - rt_frac)) * 1e3:.1f} ms (electrical.py DISC_WINDOW 40 ms)")
    P("D23", BAT54, "BAT54W", "Package_TO_SOT_SMD:SOT-323_SC-70", {1: "DISC_RC", 3: "DISC_X", 2: None},
      origin="added: reset diode, so the window measures one continuous mismatch", nl="U17")
    C("C52", "1u film", "DISC_RC", "GND", fp=C1206, nl="U17")
    R("R69", fr(rt_top), "+3V3", "DISC_REF", nl="U17")
    R("R70", fr(rt_bot), "DISC_REF", "GND", nl="U17")
    CMP2("U51", ("DISC_RC", "DISC_REF", "DISC"), ("DISC_REF", "DISC_RC", "DISC_N"), nl="U17", origin="netlist v4 (U17's comparator)")
    TP("TP25", "DISC", "TP25 DISC")

    group("Supervision: U18 12 V UVLO (10.5 V) → UV12; U19 branch overload (16 A, 5 s) → OC")
    r71 = e96(10e3 * (10.5 / 1.242 - 1))
    R("R71", fr(r71), "+12V", "V12_SNS", nl="U18", basis=f"trip {1.242 * (1 + r71 / 10e3):.2f} V (TLV3012 REF 1.242 V)")
    R("R72", "10k", "V12_SNS", "GND", nl="U18")
    P("U18", f"{PLIB}:TLV3012", "TLV3012", SOT236, {3: "U18_REF", 4: "V12_SNS", 5: "U18_REF", 1: "UV12", 6: "+3V3", 2: "GND"}, nl="U18",
      basis="IN+ = REF, IN− = 12 V divider: UV12 high below 10.5 V")
    R("R73", "10k", "+3V3", "UV12", origin="added: open-drain pull-up", nl="U18")
    DEC("U18", "+3V3")
    thr = (2.5 + 16.0 * 0.066) / 2
    r77 = e96(10e3 * thr / (VDD - thr))
    R("R74", "1M", "OC_V", "OC_RC", nl="U19", basis="÷2 + 5 s RC: (1M ∥ 1M) × 10 µF")
    R("R75", "1M", "OC_RC", "GND", nl="U19")
    C("C54", "10u", "OC_RC", "GND", fp=C1206, nl="U19")
    R("R76", "10k", "+3V3", "OC_REF", nl="U19")
    R("R77", fr(r77), "OC_REF", "GND", nl="U19", basis=f"16 A threshold: {(2 * VDD * r77 / (10e3 + r77) - 2.5) / 0.066:.1f} A (66 mV/A ESTIMATE)")
    CMP2("U53", ("OC_RC", "OC_REF", "OC"), None, nl="U19", origin="netlist v4 (U19's comparator)")
    NOTES["logic"] = ["U10 D = " + el.U10_D, "U10 CLRn = " + el.U10_CLR,
                      "K2_DRIVE = LATCH & !RUN & !PRE_TIMEOUT; FORCE = U6 (An = AUX_DB & !DISC, B = 3V3, CLRn = !K2_DRIVE).",
                      "Every logic input reads AUX_DB; only U29 (debounce) and GPIO27 read AUX_RAW.",
                      "Glue gates U40-U63 build the netlist's boolean terms (Origin = added)."]

    # ============================================================ sheet 4: base channel
    sheet("base")
    rv = dict(variant="robot")
    group("U20 G9SA-321-T075 OFF-delay safety relay (DIN rail) and U27 photoMOS (LATCH in ch1)")
    P("U20", f"{PLIB}:G9SA-321-T", f"G9SA-321-T075 set {el.BASE_SS1['t_set']:.1f}s", "",
      {"A1": "+24V_L", "A2": "GND", "T11": "U20_T11", "T12": "U20_T12", "T21": "U20_T21", "T22": "U20_T22", "T31": "U20_T31", "T32": "U20_T32",
       "T33": None, "13": None, "14": None, "23": None, "24": None, "33": None, "34": None, "41": None, "42": None,
       "47": "+24V_L", "48": "U20_D1", "57": "U20_D1", "58": "U21_EN"}, loc="chassis", nl="U20", basis="electrical.py BASE_SS1; terminals per Omron (reviewer)", **rv)
    P("U27", f"{PLIB}:AQV252G", "AQV252G", "Package_DIP:DIP-6_W7.62mm", {1: "U27_LED", 2: "GND", 3: None, 4: "S2C_U27", 5: None, 6: "U20_T12"}, nl="U27", **rv)
    R("R78", "330", "LATCH", "U27_LED", origin="added: U27 LED resistor (≈ 6 mA from LATCH)", nl="U27", **rv)

    group("Base switch Q9 / U21 LM5069-1 (soft-start, 35 A breaker): VIN → VBUS_B, enable through U20")
    P("U21", f"{PLIB}:LM5069-1", "LM5069-1", "Package_SO:MSOP-10_3x3mm_P0.5mm",
      {1: "Q9_D", 2: "VIN", 3: "U21_UV", 4: "U21_OV", 5: "GND", 6: "U21_TMR", 7: "U21_PWR", 8: None, 9: "VBUS_B", 10: "U21_GATE"}, nl="Q9, U21", **rv)
    R("R79", "1m 3W", "VIN", "Q9_D", fp=R2512, nl="Q9, U21", basis="electrical.py BASE_SWITCH '1 mΩ sense' (issue R4: 55 mV / 1 mΩ = 55 A, not 35 A)", **rv)
    MOS("Q9", "IRFB7530", "Q9_G", "Q9_D", "VBUS_B", nl="Q9, U21", **rv)
    R("R80", "10", "U21_GATE", "Q9_G", origin="added: gate resistor", nl="Q9, U21", **rv)
    R("R81", "1k", "U21_GATE", "U21_SS", origin="added: dv/dt soft-start leg", nl="Q9, U21", **rv)
    C("C55", "47n 50V", "U21_SS", "GND", origin="added: dv/dt cap, ≈ 50 ms ramp at 20 µA gate current (ESTIMATE)", nl="Q9, U21", **rv)
    C("C56", "1u", "U21_TMR", "GND", fp=C1206, origin="added: LM5069 fault timer (ESTIMATE; reviewer sizes for Q9 SOA)", nl="Q9, U21", **rv)
    R("R82", "20k", "U21_PWR", "GND", origin="added: LM5069 power limit (ESTIMATE; reviewer sizes for Q9 SOA)", nl="Q9, U21", **rv)
    R("R83", "38.3k", "U21_EN", "U21_UV", origin="added: UVLO divider: on above ≈ 12 V of U21_EN", nl="Q9, U21", **rv)
    R("R84", "10k", "U21_UV", "GND", origin="added: UVLO divider", nl="Q9, U21", **rv)
    R("R85", "102k", "VIN", "U21_OV", origin="added: OVLO divider: off above ≈ 28 V", nl="Q9, U21", **rv)
    R("R86", "10k", "U21_OV", "GND", origin="added: OVLO divider", nl="Q9, U21", **rv)
    DEC("U21", "VIN", "GND", "100n 50V", **rv)
    R("R88", "68k", "U21_EN", "BEN_L", origin="added: 24 V enable → 3.3 V logic (U25, U26)", nl="U25", **rv)
    R("R89", "10k", "BEN_L", "GND", origin="added: enable divider", nl="U25", **rv)
    P("F4", "Device:Fuse", "20A ATO", ATO, {1: "VBUS_B", 2: "BASE_OUT"}, origin="netlist v4 (block diagram 'F 20A'; no ref, issue N6)", nl="F 20A", **rv)
    STUD("J20", "BASE_OUT", "M4 stud base +", fp=STUD4, **rv)
    STUD("J21", "PGND", "M4 stud base -", fp=STUD4, **rv)
    R("R8", f"{fr(el.R_BLEED)} 3W", "VBUS_B", "PGND", fp=R3W, nl="R8", **rv)
    TP("TP16", "VBUS_B", "TP16 VBUS_B (T17-T19)", **rv)
    TP("TP17", "U21_EN", "TP17 Q9 enable (T17)", **rv)

    group("Base clamp leg: R7 NH-50 (chassis) ∥ D7, Q10; U23 window, U24 50 µs, U25 FORCE_B 100 ms → U56 → U31 (U22b) UCC27517")
    STUD("J23", "VBUS_B", "M4 stud VBUS_B (R7 lead)", fp=STUD4, **rv)
    STUD("J22", "DRN_B", "M4 stud DRN_B (R7 lead)", fp=STUD4, **rv)
    P("R7", "Device:R", f"{clamp['r_each']} NH-50", "", {1: "VBUS_B", 2: "DRN_B"}, loc="chassis", nl="Q10, R7, D7, U23, U24, U22b", **rv)
    P("D7", f"{PLIB}:VS-40CPQ060", "VS-40CPQ060-N3", TO247, {1: "DRN_B", 3: "DRN_B", 2: "VBUS_B"}, nl="Q10, R7, D7, U23, U24, U22b", **rv)
    MOS("Q10", "IRFB7530", "G_B", "DRN_B", "PGND", nl="Q10, R7, D7, U23, U24, U22b", **rv)
    R("R14", "10 1206", "BCLAMP_DRV", "G_B", fp=R1206, nl="R_G", origin="netlist v4 (R_G of the base leg)", **rv)
    R("R90", "10k", "G_B", "PGND", origin="added: gate pull-down", nl="Q10, R7, D7, U23, U24, U22b", **rv)
    C("C4", "10u 63V film", "VBUS_B", "PGND", fp=FILM10U, origin="added: local film cap for the base leg's MOSFET-diode loop (the arm leg's C2; issue N7)",
      nl="Q10, R7, D7, U23, U24, U22b", **rv)
    CMP2("U23", ("OVB_P", "VREF", "OV_B"), None, nl="Q10, R7, D7, U23, U24, U22b", **rv)
    R("R91", fr(ov["r_top"]) + " 0.1%", "VBUS_B", "OVB_P", nl="Q10, R7, D7, U23, U24, U22b", basis="as R44-R46", **rv)
    R("R92", fr(ov["r_bot"]) + " 0.1%", "OVB_P", "GND", nl="Q10, R7, D7, U23, U24, U22b", **rv)
    R("R93", fr(ov["r_hys"]) + " 0.1%", "OV_B", "OVB_P", nl="Q10, R7, D7, U23, U24, U22b", **rv)
    C("C58", fc(c_filt) + " C0G", "OVB_P", "GND", nl="Q10, R7, D7, U23, U24, U22b", basis="2 µs filter", **rv)
    ONESHOT("U24", "GND", "OV_B", "+3V3", "U24_Q", "R94", "C59", fr(r_on), fc(c_on) + " C0G", nl="Q10, R7, D7, U23, U24, U22b", basis="50 µs", **rv)
    ONESHOT("U25", "BEN_L", "+3V3", "+3V3", "FORCE_B", "R95", "C60", fr(r_force), "1u", nl="U25", basis="100 ms", **rv)
    G3("U56", "74LVC1G332", "OV_B", "U24_Q", "FORCE_B", "BCLAMP_CMD", origin=ADDED_GLUE + ": OV_B ∨ U24 ∨ FORCE_B", nl="Q10, R7, D7, U23, U24, U22b", **rv)
    DEC("U56", "+3V3", **rv)
    P("U31", f"{PLIB}:UCC27517", "UCC27517", SOT235, {1: "+12V", 2: "PGND", 3: "BCLAMP_CMD", 4: "GND", 5: "BCLAMP_DRV"},
      nl="U22b", origin="netlist v4 (as 'U22b')", **rv)
    DEC("U31", "+12V", "PGND", "1u", **rv)
    TP("TP18", "G_B", "TP18 Q10 gate (T18)", **rv)
    TP("TP19", "FORCE_B", "TP19 FORCE_B (T18)", **rv)

    group("Base watchdog U22 (GPIO23, 25 ms) → WD_B; DISC_B (U26): VBUS_B up ∧ ¬EN > 50 ms → GPIO24")
    R("R97", "100", "GPIO23", "HB_B", origin="added: series resistor on the Pi line", nl="U22", **rv)
    R("R98", "100k", "HB_B", "GND", origin="added: pull-down", nl="U22", **rv)
    ONESHOT("U22", "GND", "HB_B", "+3V3", "WD_B", "R96", "C62", fr(r_wd) + " 1%", "100n C0G", nl="U22", basis="25 ms as U5", **rv)
    TP("TP21", "WD_B", "TP21 WD_B", **rv)
    r100 = e96(100e3 * VREF / (12.0 - VREF))
    CMP2("U26", ("VBB_SNS", "VREF", "VBB_UP"), ("DISCB_RC", "DISCB_REF", "DISC_B"), nl="U26", **rv)
    R("R99", "100k", "VBUS_B", "VBB_SNS", nl="U26", basis=f"'VBUS_B up' above {VREF * (1 + 100e3 / r100):.1f} V (ESTIMATE threshold)", **rv)
    R("R100", fr(r100), "VBB_SNS", "GND", nl="U26", **rv)
    INV("U54", "74LVC1G04", "BEN_L", "BEN_N", origin=ADDED_GLUE + ": ¬EN", nl="U26", **rv)
    DEC("U54", "+3V3", **rv)
    G1("U55", "74LVC1G08", "VBB_UP", "BEN_N", "DISCB_X", origin=ADDED_GLUE, nl="U26", **rv)
    DEC("U55", "+3V3", **rv)
    r101 = e96(0.050 / 1e-6 * math.log(1 / (1 - frac)) / math.log(1 / (1 - rt_frac)))
    R("R101", fr(r101), "DISCB_X", "DISCB_RC", nl="U26", basis="50 ms RC", **rv)
    P("D24", BAT54, "BAT54W", "Package_TO_SOT_SMD:SOT-323_SC-70", {1: "DISCB_RC", 3: "DISCB_X", 2: None}, origin="added: reset diode", nl="U26", **rv)
    C("C63", "1u film", "DISCB_RC", "GND", fp=C1206, nl="U26", **rv)
    R("R102", fr(rt_top), "+3V3", "DISCB_REF", nl="U26", **rv)
    R("R103", fr(rt_bot), "DISCB_REF", "GND", nl="U26", **rv)
    R("R104", "100", "DISC_B", "GPIO24", origin="added: series resistor to the Pi", nl="U26", **rv)
    TP("TP20", "DISC_B", "TP20 DISC_B (T19)", **rv)

    group("Caster-release solenoid driver Q11 on VBUS_B (gate from the Pi, 100k pull-down)")
    MOS("Q11", "IRLML2060 60V", "Q11_G", "SOL_N", "PGND", fp=SOT23, lib="Transistor_FET:IRLML2060", nl="Q11",
        basis="'logic-level MOSFET' (no part in the netlist); 60 V for the 26.3 V clamp + flyback", **rv)
    R("R105", "100", "CASTER_REL", "Q11_G", origin="added: gate series resistor", nl="Q11", **rv)
    R("R106", "100k", "Q11_G", "PGND", nl="Q11", basis="netlist '100 kΩ pull-down'", **rv)
    P("D8", "Device:D", "1N4007", "Diode_THT:D_DO-41_SOD81_P10.16mm_Horizontal", {1: "VBUS_B", 2: "SOL_N"}, nl="Q11",
      origin="netlist v4 ('flyback diode', no ref)", **rv)
    P("J24", "Connector_Generic:Conn_01x02", "Caster solenoid", "Connector_JST:JST_VH_B2P-VH_1x02_P3.96mm_Vertical", {1: "VBUS_B", 2: "SOL_N"},
      origin="added: solenoid connector", **rv)
    NOTES["base"] = ["Robot board only: not populated on the M1 bench (Variant = robot); JP1 ties WD_B high on M1.",
                     "VBUS_B is fed from VIN by Q9, never from VBUS after K1; no diode from VBUS_B to VBUS.",
                     "U21 EN <- +24V_L - U20 47-48 - U20 57-58 (both OFF-delay NO contacts in series)."]

    # ============================================================ sheet 5: connectors and off-board devices
    sheet("conn")
    group("K1 coil + aux connector; mushrooms S1, S2 (chassis, 4 NC each) on J14 / J15")
    P("J13", "Connector_Generic:Conn_01x04", "K1 coil / aux", "Connector_JST:JST_VH_B4P-VH_1x04_P3.96mm_Vertical",
      {1: "COIL+", 2: "COIL-", 3: "AUX_A", 4: "AUX_B"}, origin="added: K1 coil + aux harness")
    P("J14", "Connector_Generic:Conn_02x04_Odd_Even", "S1 mushroom", "Connector_Molex:Molex_Micro-Fit_3.0_43045-0800_2x04_P3.00mm_Horizontal",
      {1: "+24V_L", 2: "S1A_S2A", 3: "+3V3", 4: "S1B_S2B", 5: "U20_T11", 6: "S1C_S2C", 7: "U20_T21", 8: "S1D_S2D"},
      origin="added: mushroom S1 harness (keyed Micro-Fit; swapping J14/J15 still makes a series chain)")
    P("J15", "Connector_Generic:Conn_02x04_Odd_Even", "S2 mushroom", "Connector_Molex:Molex_Micro-Fit_3.0_43045-0800_2x04_P3.00mm_Horizontal",
      {1: "S1A_S2A", 2: "COIL+", 3: "S1B_S2B", 4: "ESTOP_CH2", 5: "S1C_S2C", 6: "S2C_U27", 7: "S1D_S2D", 8: "U20_T22"},
      origin="added: mushroom S2 harness")
    P("S1", f"{PLIB}:MUSHROOM_4NC", "E-stop torso", "", {11: "+24V_L", 12: "S1A_S2A", 21: "+3V3", 22: "S1B_S2B", 31: "U20_T11", 32: "S1C_S2C",
                                                         41: "U20_T21", 42: "S1D_S2D"}, loc="chassis", nl="S1, S2", units=[1, 2, 3, 4])
    P("S2", f"{PLIB}:MUSHROOM_4NC", "E-stop base", "", {11: "S1A_S2A", 12: "COIL+", 21: "S1B_S2B", 22: "ESTOP_CH2", 31: "S1C_S2C", 32: "S2C_U27",
                                                        41: "S1D_S2D", 42: "U20_T22"}, loc="chassis", nl="S1, S2", units=[1, 2, 3, 4])
    group("ARM button SB1 (a: U10 clock, b: U20 reset), TH1 thermal switch, U20 harness")
    P("J16", "Connector_Generic:Conn_01x04", "ARM button", "Connector_Molex:Molex_Micro-Fit_3.0_43650-0400_1x04_P3.00mm_Horizontal",
      {1: "+3V3", 2: "ARM_RAW", 3: "U20_T31", 4: "U20_T32"}, origin="added: ARM button harness")
    P("SB1", f"{PLIB}:PB_2NO", "ARM / reset", "", {13: "+3V3", 14: "ARM_RAW", 23: "U20_T31", 24: "U20_T32"}, loc="chassis", nl="SB1", units=[1, 2])
    P("J17", "Connector_Generic:Conn_01x02", "TH1 thermal switch", "Connector_JST:JST_VH_B2P-VH_1x02_P3.96mm_Vertical", {1: "+3V3", 2: "TH1_OK"},
      origin="added: TH1 harness")
    P("TH1", f"{PLIB}:THERMO_NC", "85C NC", "", {1: "+3V3", 2: "TH1_OK"}, loc="chassis", nl="TH1")
    R("R53", "10k", "TH1_OK", "GND", origin="added: pull-down (open = over-temperature = STOP)", nl="TH1")
    C("C47", "100n", "TH1_OK", "GND", origin="added: cable noise filter", nl="TH1")
    P("J19", "Connector_Generic:Conn_02x06_Odd_Even", "U20 G9SA harness", "Connector_Molex:Molex_Micro-Fit_3.0_43045-1200_2x06_P3.00mm_Horizontal",
      {1: "+24V_L", 2: "GND", 3: "U20_T11", 4: "U20_T12", 5: "U20_T21", 6: "U20_T22", 7: "U20_T31", 8: "U20_T32", 9: "+24V_L", 10: "U20_D1",
       11: "U20_D1", 12: "U21_EN"}, origin="added: board ↔ U20 (DIN rail)", variant="robot")
    group("Raspberry Pi interface (heartbeats, AUX_OK, ESTOP_SENSE, DISC_B, caster release)")
    P("J18", "Connector_Generic:Conn_01x08", "Pi interface", "Connector_JST:JST_PH_B8B-PH-K_1x08_P2.00mm_Vertical",
      {1: "GPIO17", 2: "GPIO27", 3: "GPIO22", 4: "GPIO23", 5: "GPIO24", 6: "CASTER_REL", 7: "GND", 8: "GND"},
      origin="added: Pi harness (the caster-release GPIO is not named in the netlist: issue N8)")
    for i in range(1, 5):
        P(f"H{i}", "Mechanical:MountingHole", "M3", "MountingHole:MountingHole_3.2mm_M3", {}, origin="added: board mounting hole")
    NOTES["conn"] = ["Mushroom blocks: a = coil chain (+24V_L -> S1a -> S2a -> COIL+), b = logic (3V3 -> S1b -> S2b -> ESTOP_CH2),",
                     "c = U20 ch1 (T11 -> S1c -> S2c -> U27 -> T12), d = U20 ch2 (T21 -> S1d -> S2d -> T22).",
                     "Chassis parts (Location = chassis) are drawn for the net list; they connect through the board's studs and connectors."]
    return PARTS


# ------------------------------------------------------------------ libraries
class Libs:
    def __init__(self):
        self.cache = {}
        (OUT / f"{PLIB}.kicad_sym").write_text(library_text())

    def lib(self, nick):
        if nick not in self.cache:
            path = OUT / f"{PLIB}.kicad_sym" if nick == PLIB else KSYM / f"{nick}.kicad_sym"
            self.cache[nick] = Lib(path, nick)
        return self.cache[nick]

    def symbol(self, lib_id):
        nick, name = lib_id.split(":")
        return self.lib(nick).flat(name)


# ------------------------------------------------------------------ schematic writer
SHEETS = [("power", "Power path, contactor, pre-charge, logic supply"), ("clamp", "Regen clamp legs, comparator, gate drive"),
          ("logic", "Logic: watchdog, LATCH / RUN, pre-charge timer, FORCE, discrepancy, supervision"),
          ("base", "Base channel (robot board; not populated on the M1 bench)"), ("conn", "Connectors and off-board devices")]
SHEET_NAME = {"power": "Power", "clamp": "Clamp", "logic": "Logic", "base": "Base", "conn": "Connectors"}
CH = 1.27 * 0.95      # label character advance (mm), estimate for layout only
GRID = 1.27


def snap(v):
    return round(v / GRID) * GRID


def fmt(v):
    s = f"{v:.4f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def pin_dir(angle):
    """outward unit vector in schematic coordinates (y down) for a library pin angle"""
    return {0: (-1, 0), 180: (1, 0), 90: (0, 1), 270: (0, -1)}[angle]


def write_schematic(parts, libs):
    OUT.mkdir(parents=True, exist_ok=True)
    root_uuid = uid("root")
    sheet_uuid = {s: uid("sheet", s) for s, _ in SHEETS}
    sheet_of_net = {}
    for p in parts:
        for n in p.pins.values():
            if n:
                sheet_of_net.setdefault(n, set()).add(p.sheet)
    glob = set(sheet_of_net)            # every net a global label: KiCad net names = electrical.py names, no sheet prefix
    placements = {}
    for si, (sname, stitle) in enumerate(SHEETS):
        sp = [p for p in parts if p.sheet == sname]
        used = sorted({p.lib for p in sp})
        lib_syms = {lid: libs.symbol(lid) for lid in used}
        geo = {lid: Lib.units(s) for lid, s in lib_syms.items()}
        for p in sp:
            allpins = {pp["number"] for u in geo[p.lib].values() for pp in u["pins"]}
            if set(p.pins) - allpins:
                raise SystemExit(f"{p.ref}: model pins {sorted(set(p.pins) - allpins)} not on symbol {p.lib}")
            if p.units != sorted(geo[p.lib]):
                raise SystemExit(f"{p.ref}: units {p.units} but {p.lib} has {sorted(geo[p.lib])}")
        items, wires, labels, ncs, texts = [], [], [], [], []
        # ---- layout: groups as row bands
        groups = []
        for p in sp:
            if not groups or groups[-1][0] != p.group:
                groups.append((p.group, []))
            groups[-1][1].append(p)

        def layout_group(gparts, W):
            """items at local coordinates (0, 0 = top-left of the group's body area)"""
            items, x, y, row_h, xmax = [], 0.0, 0.0, 0.0, 0.0
            for p in gparts:
                for u in p.units:
                    g = geo[p.lib][u]
                    bx0, by0, bx1, by1 = g["bbox"]
                    ext = dict(l=0.0, r=0.0, t=0.0, b=0.0)
                    for pin in g["pins"]:
                        if pin["hidden"]:
                            continue
                        net = p.pins.get(pin["number"], "∅")
                        L = 2.54 + (len(net) * CH + (3.0 if net in glob else 1.0) if net not in (None, "∅") else 1.0)
                        side = {0: "l", 180: "r", 90: "b", 270: "t"}[pin["angle"]]
                        ext[side] = max(ext[side], L)
                    tw = max(len(p.ref), len(p.value)) * CH
                    side_txt = not any(pp["angle"] == 180 and not pp["hidden"] for pp in g["pins"])
                    if side_txt:
                        ext["r"] = max(ext["r"], 1.5 + tw)
                    left = -bx0 + ext["l"] + 3.0
                    width = max(left + bx1 + ext["r"] + 3.0, tw + 6.0)
                    top = by1 + ext["t"] + (2.0 if side_txt else 4.0)
                    height = top + (-by0) + ext["b"] + (2.0 if side_txt else 4.0)
                    if x + width > W and x > 0:
                        x, y = 0.0, y + row_h + 2.54
                        row_h = 0.0
                    items.append((p, u, x + left, y + top, side_txt))
                    x += width + 2.54
                    xmax = max(xmax, x)
                    row_h = max(row_h, height)
            return items, xmax, y + row_h

        def layout(Wg, ncol):
            boxes = []
            for gtitle, gparts in groups:
                it, w, h = layout_group(gparts, Wg)
                boxes.append((gtitle, it, w, h + 10.0))
            total = sum(b[3] for b in boxes)       # keep the reading order: fill column by column, balanced
            cols, acc = [[]], 0.0
            for bx in boxes:
                if acc > 0 and acc + bx[3] / 2 > total / ncol * (len(cols)) and len(cols) < ncol:
                    cols.append([])
                cols[-1].append(bx)
                acc += bx[3]
            items, texts = [], []
            x0, ymax = 25.4, 0.0
            for col in cols:
                y = 40.0
                cw = max(b[2] for b in col)
                for gtitle, it, w, h in col:
                    texts.append((gtitle, x0, y, 2.0))
                    y += 8.0
                    for (p, u, lx, ly, st) in it:
                        items.append((p, u, snap(x0 + lx), snap(y + ly), st))
                    y += h
                ymax = max(ymax, y)
                x0 += cw + 15.0
            return items, texts, ymax, x0

        notes = NOTES.get(sname, [])
        best = None
        for Wg in (180.0, 250.0, 380.0):
            for ncol in (1, 2, 3, 4):
                items_, texts_, y_, xmax_ = layout(Wg, ncol)
                H = y_ + 12 + 5 * len(notes) + 40
                Wx = xmax_ + 10
                paper_ = next(((n, w, h) for n, w, h in (("A3", 420, 297), ("A2", 594, 420), ("A1", 841, 594), ("A0", 1189, 841))
                               if h >= H and w >= Wx), None)
                if paper_ and (best is None or paper_[1] < best[0][1]):
                    best = (paper_, items_, texts_)
        if best is None:
            raise SystemExit(f"sheet {sname} too large")
        paper, items, texts = best
        # ---- symbols, stubs, labels
        sym_txt = []
        for p, u, ox, oy, side_txt in items:
            g = geo[p.lib][u]
            bx0, by0, bx1, by1 = g["bbox"]
            placements.setdefault(p.ref, []).append((sname, u, ox, oy))
            if side_txt:
                rpos, vpos = (ox + bx1 + 1.27, oy - 1.27), (ox + bx1 + 1.27, oy + 1.27)
            else:
                rpos, vpos = (ox, oy - by1 - 1.8), (ox, oy - by0 + 1.8)
            props = [("Reference", p.ref, rpos[0], rpos[1], False), ("Value", p.value, vpos[0], vpos[1], False),
                     ("Footprint", p.fp, ox, oy, True), ("Datasheet", "~", ox, oy, True),
                     ("Origin", p.origin, ox, oy, True), ("NetlistRef", p.nl or p.ref, ox, oy, True),
                     ("Location", p.loc, ox, oy, True), ("Variant", p.variant, ox, oy, True), ("Basis", p.basis, ox, oy, True)]
            on_board = "no" if (p.loc == "chassis" or not p.fp) and not p.ref.startswith("#") else "yes"
            if p.ref.startswith("#"):
                on_board = "yes"
            in_bom = "no" if p.ref.startswith("#") else "yes"
            lines = [f'(symbol (lib_id "{p.lib}") (at {fmt(ox)} {fmt(oy)} 0) (unit {u})',
                     f'  (in_bom {in_bom}) (on_board {on_board}) (dnp no)', f'  (uuid "{uid(sname, p.ref, u)}")']
            for k, v, px, py, hide in props:
                eff = '(effects (font (size 1.27 1.27))' + (' hide)' if hide else (' (justify left))' if side_txt else ')'))
                lines.append(f'  (property "{k}" {Qs(v)} (at {fmt(px)} {fmt(py)} 0) {eff})')
            for pin in g["pins"]:
                lines.append(f'  (pin "{pin["number"]}" (uuid "{uid(sname, p.ref, "pin", pin["number"])}"))')
            lines.append(f'  (instances (project "{PROJECT}" (path "/{root_uuid}/{sheet_uuid[sname]}" (reference "{p.ref}") (unit {u}))))')
            lines.append(")")
            sym_txt.append("\n".join(lines))
            seen_at = set()
            for pin in g["pins"]:
                if pin["hidden"]:
                    continue
                num = pin["number"]
                if num not in p.pins:
                    raise SystemExit(f"{p.ref} pin {num} ({pin['name']}) has no net in the model (use None for a no-connect)")
                px, py = ox + pin["x"], oy - pin["y"]
                if (round(px, 3), round(py, 3)) in seen_at:
                    continue
                seen_at.add((round(px, 3), round(py, 3)))
                net = p.pins[num]
                if net is None:
                    if pin["type"] != "no_connect":          # a no_connect-type pin needs no flag (KiCad: dangling flag)
                        ncs.append((px, py))
                    continue
                dx, dy = pin_dir(pin["angle"])
                ex, ey = px + 2.54 * dx, py + 2.54 * dy
                wires.append((px, py, ex, ey))
                ang = {(-1, 0): 180, (1, 0): 0, (0, 1): 270, (0, -1): 90}[(dx, dy)]
                labels.append((net, ex, ey, ang, net in glob))
        # ---- file
        out = ['(kicad_sch (version 20230121) (generator eeschema)', f'  (uuid "{uid("file", sname)}")', f'  (paper "{paper[0]}")',
               '  (title_block', f'    (title "Pip v4 safety board: {stitle}")', '    (date "2026-11-28")', '    (rev "netlist v4")',
               '    (company "Pip v4 home robot (R&D)")', '    (comment 1 "Generated by rnd/kicad_board.py from rnd/electrical.py (netlist v4): do not edit by hand")',
               '    (comment 2 "Not a certified safety function. Review gate G-SB: docs/rnd/safety_board_tests.md")', '  )', '  (lib_symbols']
        for lid, s in lib_syms.items():
            out.append(dump(s, 2).replace("\n", "\n    ").join(["    ", ""]))
        out.append('  )')
        for (x1, y1, x2, y2) in wires:
            out.append(f'  (wire (pts (xy {fmt(x1)} {fmt(y1)}) (xy {fmt(x2)} {fmt(y2)})) (stroke (width 0) (type default)) '
                       f'(uuid "{uid(sname, "w", fmt(x1), fmt(y1), fmt(x2), fmt(y2))}"))')
        for (x, yy) in ncs:
            out.append(f'  (no_connect (at {fmt(x)} {fmt(yy)}) (uuid "{uid(sname, "nc", fmt(x), fmt(yy))}"))')
        for i, (net, x, yy, ang, isg) in enumerate(labels):
            just = "left" if ang in (0, 90) else "right"
            if isg:
                out.append(f'  (global_label {Qs(net)} (shape passive) (at {fmt(x)} {fmt(yy)} {ang}) (fields_autoplaced)\n'
                           f'    (effects (font (size 1.27 1.27)) (justify {just}))\n    (uuid "{uid(sname, "gl", i, net)}")\n'
                           f'    (property "Intersheetrefs" "${{INTERSHEET_REFS}}" (at {fmt(x)} {fmt(yy)} 0) (effects (font (size 1.27 1.27)) hide))\n  )')
            else:
                out.append(f'  (label {Qs(net)} (at {fmt(x)} {fmt(yy)} {ang}) (fields_autoplaced)\n'
                           f'    (effects (font (size 1.27 1.27)) (justify {just} bottom))\n    (uuid "{uid(sname, "lb", i, net)}")\n  )')
        for (t, x, yy, sz) in texts:
            out.append(f'  (text {Qs(t)} (at {fmt(x)} {fmt(yy)} 0) (effects (font (size {sz} {sz}) bold) (justify left bottom)) '
                       f'(uuid "{uid(sname, "tx", t)}"))')
        if notes:
            out.append(f'  (text {Qs(chr(10).join(notes))} (at 25.4 {fmt(paper[2] - 12 - 5 * len(notes))} 0) (effects (font (size 1.8 1.8)) (justify left bottom)) '
                       f'(uuid "{uid(sname, "notes")}"))')
        out += ["  " + s.replace("\n", "\n  ") for s in sym_txt]
        out.append(")")
        (OUT / f"{sname}.kicad_sch").write_text("\n".join(out) + "\n")
    # ---- root sheet
    r = ['(kicad_sch (version 20230121) (generator eeschema)', f'  (uuid "{root_uuid}")', '  (paper "A3")',
         '  (title_block', '    (title "Pip v4 safety board (netlist v4)")', '    (date "2026-11-28")', '    (rev "netlist v4")',
         '    (company "Pip v4 home robot (R&D)")', '    (comment 1 "Generated by rnd/kicad_board.py from rnd/electrical.py: do not edit by hand")',
         '    (comment 2 "Mandatory M1 bench board + robot base channel. Not a certified safety function.")', '  )', '  (lib_symbols)']
    for i, (sname, stitle) in enumerate(SHEETS):
        x, yy = 30 + (i % 3) * 120, 50 + (i // 3) * 60
        r.append(f'  (sheet (at {x} {yy}) (size 100 30) (fields_autoplaced)\n    (stroke (width 0.1524) (type solid))\n'
                 f'    (fill (color 0 0 0 0.0000))\n    (uuid "{sheet_uuid[sname]}")\n'
                 f'    (property "Sheetname" {Qs(SHEET_NAME[sname])} (at {x} {fmt(yy - 0.7)} 0) (effects (font (size 1.27 1.27)) (justify left bottom)))\n'
                 f'    (property "Sheetfile" "{sname}.kicad_sch" (at {x} {fmt(yy + 30.6)} 0) (effects (font (size 1.27 1.27)) (justify left top)))\n'
                 f'    (instances (project "{PROJECT}" (path "/{root_uuid}" (page "{i + 2}"))))\n  )')
    intro = ("Pip v4 safety board, netlist v4 (rnd/electrical.py -> docs/rnd/electrical.md *Safety board*).\n"
             "Generated by `python -m rnd.kicad_board`; the drawing cannot drift from the analysed netlist:\n"
             "rnd/kicad_check.py compares the exported netlist with the model and with electrical.py's table.\n"
             "Fields on every symbol: Origin (netlist v4 / added: why), NetlistRef (the electrical.py ref it implements),\n"
             "Location (board / chassis), Variant (all / robot / M1), Basis (where the value comes from).\n"
             "Review: docs/rnd/safety_board_kicad.md and the G-SB checklist in docs/rnd/safety_board_tests.md.")
    r.append(f'  (text {Qs(intro)} (at 30 150 0) (effects (font (size 2 2)) (justify left bottom)) (uuid "{uid("root", "intro")}"))')
    r.append('  (sheet_instances (path "/" (page "1")))')
    r.append(")")
    (OUT / f"{PROJECT}.kicad_sch").write_text("\n".join(r) + "\n")
    return placements


def Qs(s):
    return '"' + str(s).replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n") + '"'


def write_project():
    pro = {
        "meta": {"filename": f"{PROJECT}.kicad_pro", "version": 1},
        "boards": [], "libraries": {"pinned_footprint_libs": [], "pinned_symbol_libs": []},
        "net_settings": {
            "classes": [
                {"name": "Default", "clearance": 0.2, "track_width": 0.25, "via_diameter": 0.8, "via_drill": 0.4,
                 "diff_pair_gap": 0.25, "diff_pair_via_gap": 0.25, "diff_pair_width": 0.2, "microvia_diameter": 0.3,
                 "microvia_drill": 0.1, "wire_width": 6, "bus_width": 12, "line_style": 0, "pcb_color": "rgba(0, 0, 0, 0.000)",
                 "schematic_color": "rgba(0, 0, 0, 0.000)"},
                {"name": "POWER", "clearance": 0.5, "track_width": 6.0, "via_diameter": 1.6, "via_drill": 0.8,
                 "diff_pair_gap": 0.25, "diff_pair_via_gap": 0.25, "diff_pair_width": 0.2, "microvia_diameter": 0.3,
                 "microvia_drill": 0.1, "wire_width": 6, "bus_width": 12, "line_style": 0, "pcb_color": "rgba(0, 0, 0, 0.000)",
                 "schematic_color": "rgba(0, 0, 0, 0.000)"},
                {"name": "GATE", "clearance": 0.25, "track_width": 0.8, "via_diameter": 1.0, "via_drill": 0.5,
                 "diff_pair_gap": 0.25, "diff_pair_via_gap": 0.25, "diff_pair_width": 0.2, "microvia_diameter": 0.3,
                 "microvia_drill": 0.1, "wire_width": 6, "bus_width": 12, "line_style": 0, "pcb_color": "rgba(0, 0, 0, 0.000)",
                 "schematic_color": "rgba(0, 0, 0, 0.000)"}],
            "meta": {"version": 3},
            "net_colors": None,
            "netclass_assignments": None,
            "netclass_patterns": [{"netclass": "POWER", "pattern": p} for p in POWER_NETS] +
                                 [{"netclass": "GATE", "pattern": p} for p in ("G1", "G2", "G3", "G_B", "Q4_G", "Q6_G", "Q9_G", "CLAMP_DRV", "BCLAMP_DRV")],
        },
        "schematic": {"drawing": {"default_line_thickness": 6.0, "default_text_size": 50.0}, "legacy_lib_dir": "", "legacy_lib_list": []},
        "sheets": [[uid("root"), ""]] + [[uid("sheet", s), SHEET_NAME[s]] for s, t in SHEETS],
        "text_variables": {},
    }
    (OUT / f"{PROJECT}.kicad_pro").write_text(json.dumps(pro, indent=2) + "\n")
    libs = sorted({p.lib.split(":")[0] for p in PARTS})
    sl = ["(sym_lib_table"]
    for nick in libs:
        uri = f"${{KIPRJMOD}}/{PLIB}.kicad_sym" if nick == PLIB else f"${{KICAD7_SYMBOL_DIR}}/{nick}.kicad_sym"
        sl.append(f'  (lib (name "{nick}")(type "KiCad")(uri "{uri}")(options "")(descr ""))')
    sl.append(")")
    (OUT / "sym-lib-table").write_text("\n".join(sl) + "\n")
    fls = sorted({p.fp.split(":")[0] for p in PARTS if p.fp})
    fl = ["(fp_lib_table"] + [f'  (lib (name "{n}")(type "KiCad")(uri "${{KICAD7_FOOTPRINT_DIR}}/{n}.pretty")(options "")(descr ""))' for n in fls] + [")"]
    (OUT / "fp-lib-table").write_text("\n".join(fl) + "\n")


POWER_NETS = ["PSU+", "F1_OUT", "VIN", "VBUS", "PGND", "DRN1", "DRN2", "DRN3", "ARM_F", "ARM_LEAD", "K2_C", "Q9_D", "VBUS_B", "BASE_OUT", "DRN_B"]


def model_json(parts):
    return {"parts": [dict(ref=p.ref, lib=p.lib, value=p.value, fp=p.fp, sheet=p.sheet, pins=p.pins, origin=p.origin, nl=p.nl or p.ref,
                           loc=p.loc, variant=p.variant, basis=p.basis, group=p.group) for p in parts if not p.ref.startswith("#")],
            "power_nets": POWER_NETS,
            "flags": sorted(n for p in parts if p.lib == "power:PWR_FLAG" for n in p.pins.values())}


# ------------------------------------------------------------------ exports
def kicad_cli(*args):
    r = subprocess.run(["kicad-cli", *args], capture_output=True, text=True)
    if r.returncode != 0:
        raise SystemExit(f"kicad-cli {' '.join(args)} failed:\n{r.stdout}\n{r.stderr}")
    return r.stdout + r.stderr


def exports():
    sch = str(OUT / f"{PROJECT}.kicad_sch")
    kicad_cli("sch", "export", "netlist", "--format", "kicadsexpr", "-o", str(OUT / f"{PROJECT}.net"), sch)
    t = (OUT / f"{PROJECT}.net").read_text()             # reproducible file: no absolute path or wall-clock date
    t = re.sub(r'\(source "[^"]*"\)', f'(source "{PROJECT}.kicad_sch")', t, count=1)
    t = re.sub(r'\(date "[^"]*"\)', '(date "generated")', t, count=1)
    (OUT / f"{PROJECT}.net").write_text(t)
    # the board netlist (.net) leaves out the chassis parts (on_board no); the BOM export keeps every part and the full nets
    kicad_cli("sch", "export", "python-bom", "-o", str(OUT / f"{PROJECT}_full_netlist.xml"), sch)
    t = (OUT / f"{PROJECT}_full_netlist.xml").read_text()
    t = re.sub(r"<source>[^<]*</source>", f"<source>{PROJECT}.kicad_sch</source>", t, count=1)
    t = re.sub(r"<date>[^<]*</date>", "<date>generated</date>", t, count=1)
    (OUT / f"{PROJECT}_full_netlist.xml").write_text(t)
    kicad_cli("sch", "export", "pdf", "-o", str(OUT / f"{PROJECT}_schematic.pdf"), sch)
    bom_csv(OUT / f"{PROJECT}_full_netlist.xml", OUT / f"{PROJECT}_bom.csv")


def bom_csv(xml_path, csv_path):
    import xml.etree.ElementTree as ET
    t = ET.parse(xml_path)
    rows = {}
    for c in t.getroot().iter("comp"):
        ref = c.get("ref")
        f = {x.get("name"): (x.text or "") for x in c.iter("field")}
        val = (c.findtext("value") or "")
        fp = (c.findtext("footprint") or "")
        key = (val, fp, f.get("Location", ""), f.get("Variant", ""))
        r = rows.setdefault(key, dict(refs=[], value=val, footprint=fp, location=f.get("Location", ""), variant=f.get("Variant", ""),
                                      origin=set(), netlist=set()))
        r["refs"].append(ref)
        r["origin"].add(f.get("Origin", "").split(":")[0])
        r["netlist"].add(f.get("NetlistRef", ""))

    def natural(s):
        return [int(t) if t.isdigit() else t for t in re.split(r"(\d+)", s)]
    with open(csv_path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["Qty", "References", "Value", "Footprint", "Location", "Variant", "Origin", "Netlist ref(s)"])
        for r in sorted(rows.values(), key=lambda r: natural(sorted(r["refs"], key=natural)[0])):
            refs = sorted(r["refs"], key=natural)
            w.writerow([len(refs), " ".join(refs), r["value"], r["footprint"], r["location"], r["variant"], "/".join(sorted(r["origin"])),
                        "; ".join(sorted(r["netlist"]))])


def main(argv=None):
    argv = argv or sys.argv[1:]
    parts = design()
    libs = Libs()
    (OUT / "out").mkdir(parents=True, exist_ok=True)
    write_schematic(parts, libs)
    write_project()
    (OUT / "out" / "model.json").write_text(json.dumps(model_json(parts), indent=1, ensure_ascii=False) + "\n")
    exports()
    from . import kicad_check
    bad = kicad_check.main([])
    if "--pcb" in argv:
        r = subprocess.run(["/usr/bin/python3", "-m", "rnd.kicad_pcb"], cwd=ROOT)
        if r.returncode:
            raise SystemExit("rnd.kicad_pcb failed")
    n_board = sum(1 for p in parts if p.loc == "board" and not p.ref.startswith("#"))
    if bad:
        print(f"kicad_check: {bad} problems (see hardware/safety_board/safety_board_crosscheck.txt / _erc.txt)")
    print(f"safety board: {len(parts)} symbols ({n_board} on the board), {len({n for p in parts for n in p.pins.values() if n})} nets → {OUT}")


if __name__ == "__main__":
    main()
