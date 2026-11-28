"""Checks on the generated safety-board KiCad project (hardware/safety_board/), run by rnd/kicad_board.py.

    python -m rnd.kicad_check

1. **Drawing = model, net for net:** the netlist KiCad exports from the schematic (kicad-cli, safety_board.net) against
   the generator's model (out/model.json): every part, value, footprint, and every pin's net, both ways.
2. **Model = electrical.py:** every row of electrical.md's *Safety board* ref table (rnd/electrical.py
   `safety_board_lines()`, parsed fresh, not cached) is drawn: its refs exist (NetlistRef field), and every net the row
   names touches the row's parts, directly or through added glue (gates, series / pull resistors, the gate drivers, the
   net ties); plus the block diagram's chains that the race fix and the stop path depend on.
3. **Netlist ERC:** KiCad 7.0.11's kicad-cli has no `sch erc` (added in KiCad 8), so this applies KiCad's default
   pin-conflict matrix, the driven-input / driven-power-pin rules and the unconnected-pin rule to the exported netlist.
Output: hardware/safety_board/safety_board_crosscheck.txt and safety_board_erc.txt.
"""
from __future__ import annotations

import json
import re
import sys
from collections import defaultdict, deque
from pathlib import Path

from .kicad_sexp import find, find_all, parse

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "hardware" / "safety_board"

# KiCad 7 default ERC pin map (eeschema/erc_settings.cpp), order below; 0 OK, 1 warning, 2 error
TYPES = ["input", "output", "bidirectional", "tri_state", "passive", "unspecified", "power_in", "power_out", "open_collector",
         "open_emitter", "no_connect"]
PINMAP = [[0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 2],
          [0, 2, 0, 1, 0, 1, 0, 2, 2, 2, 2],
          [0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 2],
          [0, 1, 0, 0, 0, 1, 1, 2, 1, 1, 2],
          [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 2],
          [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2],
          [0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 2],
          [0, 2, 1, 2, 0, 1, 0, 2, 2, 2, 2],
          [0, 2, 0, 1, 0, 1, 0, 2, 0, 0, 2],
          [0, 2, 1, 1, 0, 1, 0, 2, 0, 0, 2],
          [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]]
DRIVERS = {"output", "bidirectional", "tri_state", "passive", "power_out", "open_collector", "open_emitter"}


def read_netlist(path):
    root = parse(Path(path).read_text())
    comps = {}
    for c in find_all(find(root, "components"), "comp"):
        ref = str(find(c, "ref")[1])
        flds = {str(f[1][1]): str(f[2]) for f in find_all(find(c, "fields") or [], "field") if len(f) > 2}
        comps[ref] = dict(value=str(find(c, "value")[1]), fp=str((find(c, "footprint") or [None, ""])[1]), fields=flds)
    nets = {}
    for n in find_all(find(root, "nets"), "net"):
        name = str(find(n, "name")[1])
        nodes = [(str(find(x, "ref")[1]), str(find(x, "pin")[1]), str(find(x, "pintype")[1])) for x in find_all(n, "node")]
        nets[name] = nodes
    return comps, nets


def read_xml(path):
    """the full netlist (kicad-cli sch export python-bom): every part, chassis ones included, and every net"""
    import xml.etree.ElementTree as ET
    r = ET.parse(path).getroot()
    comps = {}
    for c in r.iter("comp"):
        flds = {f.get("name"): (f.text or "") for f in c.iter("field")}
        comps[c.get("ref")] = dict(value=c.findtext("value") or "", fp=c.findtext("footprint") or "", fields=flds)
    nets = {n.get("name"): [(x.get("ref"), x.get("pin"), x.get("pintype")) for x in n.iter("node")] for n in r.iter("net")}
    return comps, nets


def norm(name):
    return name.rsplit("/", 1)[-1] if not name.startswith("unconnected-") else name


# ------------------------------------------------------------------ 1. drawing vs model
def drawing_vs_model(model, comps, nets):
    L, bad = [], 0
    parts = {p["ref"]: p for p in model["parts"]}
    pin_net = {}
    for name, nodes in nets.items():
        for ref, pin, _ in nodes:
            pin_net[(ref, pin)] = norm(name)
    for ref in sorted(set(parts) | set(comps)):
        if ref not in comps:
            L.append(f"  MISSING in drawing: {ref}")
            bad += 1
            continue
        if ref not in parts:
            L.append(f"  EXTRA in drawing: {ref}")
            bad += 1
            continue
        p, c = parts[ref], comps[ref]
        if p["value"] != c["value"] or p["fp"] != c["fp"]:
            L.append(f"  {ref}: value/footprint {c['value']!r}/{c['fp']!r} vs model {p['value']!r}/{p['fp']!r}")
            bad += 1
        for pin, net in p["pins"].items():
            got = pin_net.get((ref, pin))
            if net is None:
                if not (got or "").startswith("unconnected-"):
                    L.append(f"  {ref}.{pin}: model no-connect, drawing net {got}")
                    bad += 1
            elif got != net:
                L.append(f"  {ref}.{pin}: model {net}, drawing {got}")
                bad += 1
        for (r, pin), got in pin_net.items():
            if r == ref and pin not in p["pins"]:
                L.append(f"  {ref}.{pin}: in the drawing ({got}) but not in the model")
                bad += 1
    m_nets = defaultdict(set)
    for p in model["parts"]:
        for pin, net in p["pins"].items():
            if net:
                m_nets[net].add((p["ref"], pin))
    d_nets = {norm(n): {(r, p) for r, p, _ in nodes} for n, nodes in nets.items() if not n.startswith("unconnected-")}
    same = sum(1 for n in m_nets if d_nets.get(n) == m_nets[n])
    for n in sorted(set(m_nets) | set(d_nets)):
        if d_nets.get(n) != m_nets.get(n):
            L.append(f"  net {n}: drawing {sorted(d_nets.get(n, []))} vs model {sorted(m_nets.get(n, []))}")
            bad += 1
    head = (f"1. Drawing vs model: {len(comps)} components, {len(d_nets)} named nets in the exported full netlist; "
            f"{same}/{len(m_nets)} model nets identical node-for-node; {bad} differences")
    return [head] + L, bad


# ------------------------------------------------------------------ 2. model vs electrical.py
ALIAS = {"3V3": "+3V3", "COIL−": "COIL-", "COIL+": "COIL+", "+24V_L": "+24V_L", "PSU+": "PSU+", "STOP": "RUN_OK", "POR": "LATCH_CLRn",
         "AUX_OK": "GPIO27", "ESTOP_SENSE": "GPIO22", "S1b": "ESTOP_CH2", "S2b": "ESTOP_CH2", "TH1": "TH1_OK", "WD_B": "WD_B",
         "EN": "U21_EN", "SB1": "ARM_RAW", "SB1b": "U20_T31", "S1a": "S1A_S2A", "S2a": "COIL+", "S1c": "S1C_S2C", "S2c": "S2C_U27",
         "S1d": "S1D_S2D", "S2d": "U20_T22", "GND": "GND"}
GND_SET = {"GND", "PGND"}
NOT_NETS = {"IRFB7530", "ALEV100", "RH", "NH", "ATO", "AWG", "XOR", "AND", "OR", "NC", "NO", "LED", "BMS", "SMDJ33A", "TLV3202",
            "UCC27524", "UCC27517", "LM74700", "LM5069", "ACS724", "PC817", "BAT54", "C0G", "RC", "Q", "D", "CLK", "B", "U2", "F3",
            "R3", "R7", "Q10", "Q4", "U6", "U10", "U13", "U21", "U27", "U8", "U9", "Q1", "Q3", "F2", "K2", "R", "VBUS_B divider",
            "Ā", "CLR̄", "Pi", "DISC_B"}


def expand_refs(cell):
    out = set()
    for part in re.split(r"[,/]\s*", cell):
        part = part.strip()
        m = re.match(r"([A-Z]+)(\d+)–[A-Z]*(\d+)$", part)
        if m:
            out |= {f"{m.group(1)}{i}" for i in range(int(m.group(2)), int(m.group(3)) + 1)}
        elif part:
            out.add(part)
    return out


def graph(model):
    """net adjacency through 'transparent' parts: added glue / passives, the gate drivers, net ties, inverters"""
    adj = defaultdict(set)
    for p in model["parts"]:
        transparent = (p["origin"].startswith("added") or p["ref"] in ("U9", "U31", "U15", "U30", "U58", "U57", "U19", "U27")
                       or p["lib"].startswith("Device:NetTie") or p["nl"] == "R_G")
        if not transparent:
            continue
        nets = {n for n in p["pins"].values() if n and n not in ("GND", "PGND", "+3V3", "+12V", "+5V")}
        for a in nets:
            for b in nets:
                if a != b:
                    adj[a].add((b, p["ref"]))
    return adj


def reach(adj, start, targets, depth=4):
    seen, dq = {start: []}, deque([start])
    while dq:
        n = dq.popleft()
        if n in targets:
            return seen[n]
        if len(seen[n]) >= depth:
            continue
        for m, via in adj[n]:
            if m not in seen:
                seen[m] = seen[n] + [via]
                dq.append(m)
    return None


def model_vs_electrical(model, rows):
    parts = model["parts"]
    nets_all = {n for p in parts for n in p["pins"].values() if n}
    adj = graph(model)
    L, bad, n_claims, n_ok = [], 0, 0, 0
    for cell, (part, nets_txt, func) in rows.items():
        refs = expand_refs(cell)
        mine = [p for p in parts if p["nl"] == cell or p["ref"] in refs or p["nl"] in refs
                or any(p["nl"] == r for r in refs) or (p["ref"] == "R5" and "R3" in refs and "pre-charge" in func)]
        if cell == "R3":
            mine = [p for p in parts if p["ref"] == "R5"]
        if cell == "S1c/d, S2c/d":
            mine = [p for p in parts if p["ref"] in ("S1", "S2")]
        if not mine:
            L.append(f"  row {cell}: NOT DRAWN")
            bad += 1
            continue
        touch = {n for p in mine for n in p["pins"].values() if n}
        toks = re.findall(r"\+?[A-Za-z][A-Za-z0-9_]*[+−]?(?:_[A-Z0-9]+)*", nets_txt.replace("**", ""))
        res = []
        for t in toks:
            net = ALIAS.get(t, t)
            if t in NOT_NETS or net not in nets_all and net not in GND_SET:
                continue
            n_claims += 1
            targets = GND_SET if net in GND_SET else {net, net + "_N"}     # X_N: the complement output of the same part
            if touch & targets:
                n_ok += 1
                res.append(f"{t}=direct" + ("" if net in touch or net in GND_SET else f" (as {net}_N)"))
                continue
            path = None
            for tn in touch:
                path = reach(adj, tn, targets)
                if path is not None:
                    break
            if path is not None:
                n_ok += 1
                res.append(f"{t}=via {'+'.join(path)}")
            else:
                bad += 1
                res.append(f"{t}=NOT CONNECTED")
        drawn = ", ".join(sorted({p['ref'] for p in mine}, key=lambda r: (re.sub(r'\d', '', r), int(re.sub(r'\D', '', r) or 0))))
        L.append(f"  {cell:<28} drawn as {drawn}")
        if res:
            L.append(f"  {'':<28} nets: " + "; ".join(dict.fromkeys(res)))
    head = f"2. Model vs electrical.py ref table: {len(rows)} rows, {n_claims} net claims, {n_ok} satisfied, {bad} failures"
    return [head] + L, bad


CHAINS = [  # (text in electrical.py's block diagram, [(ref, pin, net), ...]) — the topology the stop path relies on
    ("+24V_L ─ S1a NC ─ S2a NC ─ K1 coil (D1 + Z1 suppression) ─ Q4 drain",
     [("S1", "11", "+24V_L"), ("S1", "12", "S1A_S2A"), ("S2", "11", "S1A_S2A"), ("S2", "12", "COIL+"), ("K1", "X1", "COIL+"),
      ("K1", "X2", "COIL-"), ("Q4", "2", "COIL-"), ("D1", "1", "COIL+"), ("Z1", "1", "COIL-")]),
    ("+24V_L ─ S1a/S2a (same chain) ─ K2 coil ─ Q5", [("K2", "5", "COIL+"), ("K2", "2", "K2_COIL-"), ("Q5", "3", "K2_COIL-")]),
    ("K2_DRIVE = LATCH ∧ ¬RUN ∧ ¬PRE_TIMEOUT", [("U47", "1", "LATCH"), ("U47", "2", "RUN_N"), ("U28", "3", "RUN_N"),
                                                ("U46", "1", "K2X"), ("U46", "2", "PRE_TO_N"), ("U46", "4", "K2_DRIVE"), ("Q5", "1", "Q5_G")]),
    ("U28 SN74LVC1G74: D = 3V3, CLK = AUX_DB ↑, CLR̄ = LATCH", [("U28", "2", "+3V3"), ("U28", "1", "AUX_DB"), ("U28", "6", "LATCH"),
                                                              ("U28", "5", "RUN")]),
    ("K1 aux (NO) ─ U12 PC817 ─ AUX_RAW (TP8)", [("K1", "3", "AUX_A"), ("K1", "4", "AUX_B"), ("U12", "1", "AUX_B"), ("TP8", "1", "AUX_RAW")]),
    ("R_c 100k ∥ (BAT54 + R_d 4.7k) ─ C_db 47nF C0G ─ U29 74LVC1G17 → AUX_DB (TP12)",
     [("R20", "1", "AUX_RAW"), ("R20", "2", "AUX_C"), ("D20", "1", "AUX_C"), ("R21", "2", "AUX_RAW"), ("C20", "1", "AUX_C"),
      ("U29", "2", "AUX_C"), ("U29", "4", "AUX_DB"), ("TP12", "1", "AUX_DB")]),
    ("FORCE = U6 one-shot, Ā = AUX_DB (falling edge), B = 3V3, CLR̄ = ¬K2_DRIVE (U15)",
     [("U6", "1", "U6_TRIG"), ("U52", "1", "AUX_DB"), ("U6", "2", "+3V3"), ("U6", "3", "U6_CLRn"), ("U15", "2", "K2_DRIVE"),
      ("U15", "4", "U6_CLRn"), ("U6", "5", "FORCE")]),
    ("U7 OR( OV , FORCE ) → U9 UCC27524 → R_G → Q1–Q3", [("U7", "1", "OV"), ("U7", "6", "FORCE"), ("U7", "3", "U16_Q"), ("U9", "2", "CLAMP_CMD"),
                                                       ("U9", "7", "CLAMP_DRV"), ("R11", "2", "G1"), ("Q1", "1", "G1"), ("Q2", "1", "G2"), ("Q3", "1", "G3")]),
    ("GPIO17 heartbeat ─ U5a 74LVC1G17 ─ U5 SN74LVC1G123 B (rising edge), Ā = GND, CLR̄ = 3V3 (R4·C3) → WD_OK",
     [("U30", "4", "HB_BUF"), ("U5", "2", "HB_BUF"), ("U5", "1", "GND"), ("U5", "3", "+3V3"), ("U5", "5", "WD_OK")]),
    ("U17 XOR(Q4 gate, AUX_DB) > 40 ms", [("U17", "1", "U8_OUT"), ("U17", "2", "AUX_DB"), ("U51", "1", "DISC")]),
    ("VIN ─ Q9 IRFB7530 (U21 LM5069-1: soft-start, 35 A breaker) ─ VBUS_B", [("R79", "1", "VIN"), ("Q9", "2", "Q9_D"), ("Q9", "3", "VBUS_B")]),
    ("U21 EN ← +24V_L ─ U20 delayed contact 1 ─ U20 delayed contact 2", [("U20", "47", "+24V_L"), ("U20", "48", "U20_D1"),
                                                                       ("U20", "57", "U20_D1"), ("U20", "58", "U21_EN")]),
    ("ch1 = S1c ─ S2c ─ U27 photoMOS (LATCH), ch2 = S1d ─ S2d; reset = SB1b",
     [("S1", "31", "U20_T11"), ("S2", "32", "S2C_U27"), ("U27", "4", "S2C_U27"), ("U27", "6", "U20_T12"), ("S1", "41", "U20_T21"),
      ("S2", "42", "U20_T22"), ("SB1", "23", "U20_T31"), ("SB1", "24", "U20_T32")]),
    ("FORCE_B:                    U25 SN74LVC1G123 100 ms one-shot on U21 EN falling → Q10 gate",
     [("U25", "1", "BEN_L"), ("U25", "5", "FORCE_B"), ("U56", "6", "FORCE_B"), ("U31", "5", "BCLAMP_DRV"), ("Q10", "1", "G_B")]),
    ("R7 0.56Ω NH-50 ∥ D7 VS-40CPQ060 ── Q10 IRFB7530 ── GND", [("R7", "1", "VBUS_B"), ("R7", "2", "DRN_B"), ("D7", "1", "DRN_B"),
                                                              ("D7", "2", "VBUS_B"), ("Q10", "2", "DRN_B"), ("Q10", "3", "PGND")]),
]


def chains(model, art):
    pins = {(p["ref"], k): v for p in model["parts"] for k, v in p["pins"].items()}
    L, bad = [], 0
    for src, exp in CHAINS:
        stale = src not in art
        miss = [f"{r}.{pin}={pins.get((r, pin))} (want {n})" for r, pin, n in exp if pins.get((r, pin)) != n]
        st = "STALE (text no longer in electrical.py)" if stale else ("ok" if not miss else "FAIL " + ", ".join(miss))
        bad += bool(stale or miss)
        L.append(f"  [{st}] {src[:110]}")
    return [f"3. Block-diagram chains: {len(CHAINS)} checked, {bad} failures"] + L, bad


# ------------------------------------------------------------------ 4. netlist ERC
def erc(model, nets):
    flags = {n for p in model.get("flags", []) for n in [p]}
    errs, warns = [], []
    for name, nodes in nets.items():
        n = norm(name)
        types = [(r, p, t.replace("+no_connect", ""), t.endswith("+no_connect")) for r, p, t in nodes]
        if name.startswith("unconnected-"):
            for r, p, t, nc in types:
                if not nc and t != "no_connect":
                    errs.append(f"pin not connected: {r}.{p} ({t})")
            continue
        if len(types) == 1:
            warns.append(f"single-pin net {n}: {types[0][0]}.{types[0][1]}")
        for i in range(len(types)):
            for j in range(i + 1, len(types)):
                a, b = TYPES.index(types[i][2] if types[i][2] != "free" else "unspecified"), TYPES.index(types[j][2] if types[j][2] != "free" else "unspecified")
                lvl = PINMAP[a][b]
                msg = f"pin conflict on {n}: {types[i][0]}.{types[i][1]} ({types[i][2]}) - {types[j][0]}.{types[j][1]} ({types[j][2]})"
                if lvl == 2:
                    errs.append(msg)
                elif lvl == 1:
                    warns.append(msg)
        ts = {t for _, _, t, _ in types}
        if "input" in ts and not ts & DRIVERS:
            errs.append(f"input pin not driven on {n}")
        if "power_in" in ts and "power_out" not in ts and n not in flags:
            errs.append(f"power input not driven (no power_out or PWR_FLAG) on {n}")
    return errs, warns


def main(argv=None):
    model = json.loads((OUT / "out" / "model.json").read_text())
    comps, nets = read_xml(OUT / "safety_board_full_netlist.xml")
    bcomps, bnets = read_netlist(OUT / "safety_board.net")
    board = {p["ref"] for p in model["parts"] if p["loc"] == "board"}
    sb = [f"   board netlist (safety_board.net, for the PCB): {len(bcomps)} footprints; "
          + ("= the model's on-board parts" if set(bcomps) == board else f"MISMATCH {sorted(set(bcomps) ^ board)}")]
    from .kicad_board import netlist_rows, ART, VALUES, design
    if not VALUES:
        design()
    rows = netlist_rows()
    s1, b1 = drawing_vs_model(model, comps, nets)
    s2, b2 = model_vs_electrical(model, rows)
    s3, b3 = chains(model, ART)
    errs, warns = erc(model, bnets)
    b1 += set(bcomps) != board
    txt = ["Safety board KiCad cross-check (python -m rnd.kicad_check; generated, do not edit)", ""] + s1[:1] + sb + s1[1:] + [""] + s2 + [""] + s3 + [""]
    added = [p for p in model["parts"] if p["origin"].startswith("added")]
    txt += [f"Parts: {len(model['parts'])} drawn, {len(model['parts']) - len(added)} from netlist v4, {len(added)} added by the drawing "
            f"(glue gates, pull / series resistors, decoupling, connectors, test points: the Origin field gives each reason)."]
    txt += ["", "5. Drawn values vs electrical.py (values follow electrical.py's constants; rnd/safety_board_sim.py models these)"]
    txt += [f"  {a:<40} electrical.py: {b:<44} drawn: {c}" for a, b, c in VALUES]
    (OUT / "safety_board_crosscheck.txt").write_text("\n".join(txt) + "\n")
    e = ["Netlist ERC (KiCad 7 default pin map applied to the kicad-cli netlist; kicad-cli 7.0.11 has no `sch erc`)", "",
         f"errors: {len(errs)}", *[f"  E {x}" for x in errs], f"warnings: {len(warns)}", *[f"  W {x}" for x in warns]]
    (OUT / "safety_board_erc.txt").write_text("\n".join(e) + "\n")
    print(s1[0])
    print(s2[0])
    print(s3[0])
    print(f"4. Netlist ERC: {len(errs)} errors, {len(warns)} warnings")
    return b1 + b2 + b3 + len(errs)


if __name__ == "__main__":
    sys.exit(1 if main() else 0)
