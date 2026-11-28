"""Project symbol library for the safety board (hardware/safety_board/pip_safety.kicad_sym).

Parts that the KiCad 7 standard libraries do not carry: two are copies of a library symbol with the same pinout
(TLV3202 = the LM2903 dual-comparator body with push-pull outputs; 74LVC1G74 = the 74AUC1G74 body, the same DCU
pinout), the rest are drawn here as pin boxes with the datasheet pin numbers. Every pinout below is from memory of
the datasheet and is a G-SB review item (docs/rnd/safety_board_kicad.md, *Reviewer: verify*).
"""
from __future__ import annotations

from .kicad_sexp import Lib, Q, dump

KSYM = "/usr/share/kicad/symbols"

FONT = "(effects (font (size 1.27 1.27)))"
FONT_H = "(effects (font (size 1.27 1.27)) hide)"


def _props(ref, value, fp, desc, kw="", filt=""):
    s = [f'(property "Reference" "{ref}" (at 0 1.27 0) {FONT})',
         f'(property "Value" "{value}" (at 0 -1.27 0) {FONT})',
         f'(property "Footprint" "{fp}" (at 0 0 0) {FONT_H})',
         f'(property "Datasheet" "~" (at 0 0 0) {FONT_H})',
         f'(property "ki_description" "{desc}" (at 0 0 0) {FONT_H})']
    if kw:
        s.append(f'(property "ki_keywords" "{kw}" (at 0 0 0) {FONT_H})')
    if filt:
        s.append(f'(property "ki_fp_filters" "{filt}" (at 0 0 0) {FONT_H})')
    return s


def _pin(num, name, etype, x, y, ang, length=2.54):
    return (f'(pin {etype} line (at {x:g} {y:g} {ang}) (length {length:g}) (name "{name}" {FONT}) '
            f'(number "{num}" {FONT}))')


def _box_unit(name, unit, sides, min_w=10.16):
    """sides: dict left/right/top/bottom -> list of (num, name, type) or None for a gap. Returns sub-symbol text."""
    L, R = sides.get("left", []), sides.get("right", [])
    T, B = sides.get("top", []), sides.get("bottom", [])
    rows = max(len(L), len(R), 1)
    cols = max(len(T), len(B), 1)
    lab = max([len(p[1]) for p in L + R if p] + [2])
    w = max(min_w, 2.54 * (2 * round(((lab * 1.27 * 0.9) * 2 + 2.54) / 5.08 + 0.5)), 2.54 * (cols + 1))
    w = 2.54 * round(w / 2.54)
    if round(w / 2.54) % 2:
        w += 2.54
    h = 2.54 * (rows + 1)
    if round(h / 2.54) % 2:
        h += 2.54
    hw, hh = w / 2, h / 2
    out = [f'(symbol "{name}_{unit}_1"',
           f'(rectangle (start {-hw:g} {hh:g}) (end {hw:g} {-hh:g}) (stroke (width 0.254) (type default)) (fill (type background)))']

    def col(lst, x, ang):
        y0 = hh - 2.54
        for i, p in enumerate(lst):
            if p:
                out.append(_pin(p[0], p[1], p[2], x, y0 - 2.54 * i, ang))

    def row(lst, y, ang):
        x0 = -2.54 * (len(lst) - 1) / 2
        x0 = 2.54 * round(x0 / 2.54)
        for i, p in enumerate(lst):
            if p:
                out.append(_pin(p[0], p[1], p[2], x0 + 2.54 * i, y, ang))

    col(L, -hw - 2.54, 0)
    col(R, hw + 2.54, 180)
    row(T, hh + 2.54, 270)
    row(B, -hh - 2.54, 90)
    out.append(")")
    return "\n".join(out)


def box_symbol(name, ref, value, fp, desc, units, kw="", filt="", pin_names_offset=1.016):
    s = [f'(symbol "{name}" (pin_names (offset {pin_names_offset})) (in_bom yes) (on_board yes)']
    s += _props(ref, value, fp, desc, kw, filt)
    for i, u in enumerate(units, 1):
        s.append(_box_unit(name, i, u))
    s.append(")")
    return "\n".join(s)


def contact_symbol(name, ref, value, desc, units, normally="closed"):
    """Multi-unit dry contact (mushroom NC blocks, ARM NO blocks, thermal switch). units: [(n1, n2), ...]."""
    s = [f'(symbol "{name}" (pin_names (offset 0) hide) (in_bom yes) (on_board no)']
    s += _props(ref, value, "", desc)
    for i, (a, b) in enumerate(units, 1):
        g = [f'(symbol "{name}_{i}_1"',
             '(circle (center -2.032 0) (radius 0.508) (stroke (width 0) (type default)) (fill (type none)))',
             '(circle (center 2.032 0) (radius 0.508) (stroke (width 0) (type default)) (fill (type none)))']
        if normally == "closed":
            g.append('(polyline (pts (xy -1.524 0.254) (xy 2.54 -0.762)) (stroke (width 0.254) (type default)) (fill (type none)))')
            g.append('(polyline (pts (xy 0 -0.508) (xy 0 2.032)) (stroke (width 0.254) (type dash)) (fill (type none)))')
        else:
            g.append('(polyline (pts (xy -1.524 0.254) (xy 2.54 1.778)) (stroke (width 0.254) (type default)) (fill (type none)))')
            g.append('(polyline (pts (xy 0 1.016) (xy 0 3.048)) (stroke (width 0.254) (type dash)) (fill (type none)))')
        g.append(_pin(a, str(a), "passive", -5.08, 0, 0, 2.54))
        g.append(_pin(b, str(b), "passive", 5.08, 0, 180, 2.54))
        g.append(")")
        s.append("\n".join(g))
    s.append(")")
    return "\n".join(s)


def diode_dual_symbol():
    name = "VS-40CPQ060"
    s = [f'(symbol "{name}" (pin_names (offset 1.016) hide) (in_bom yes) (on_board yes)']
    s += _props("D", name, "Package_TO_SOT_THT:TO-247-3_Vertical",
                "Vishay VS-40CPQ060-N3, 60 V 2x20 A common-cathode Schottky, TO-247AC; both anodes in parallel (pin 1, 3 = A, 2 + tab = K)",
                "schottky dual common cathode", "TO?247*")
    s.append(f'(symbol "{name}_0_1"\n'
             '(polyline (pts (xy -1.27 1.27) (xy -1.27 -1.27) (xy 1.27 0) (xy -1.27 1.27)) (stroke (width 0.254) (type default)) (fill (type none)))\n'
             '(polyline (pts (xy 1.905 1.27) (xy 1.905 1.778) (xy 1.27 1.778) (xy 1.27 -1.778) (xy 0.635 -1.778) (xy 0.635 -1.27)) '
             '(stroke (width 0.254) (type default)) (fill (type none)))\n'
             '(polyline (pts (xy -2.54 2.54) (xy -2.54 -2.54)) (stroke (width 0) (type default)) (fill (type none)))\n'
             '(polyline (pts (xy -2.54 0) (xy -1.27 0)) (stroke (width 0) (type default)) (fill (type none)))\n'
             '(polyline (pts (xy 1.27 0) (xy 2.54 0)) (stroke (width 0) (type default)) (fill (type none)))\n)')
    s.append(f'(symbol "{name}_1_1"\n' + _pin("1", "A", "passive", -5.08, 2.54, 0) + "\n"
             + _pin("3", "A", "passive", -5.08, -2.54, 0) + "\n" + _pin("2", "K", "passive", 5.08, 0, 180) + "\n)")
    s.append(")")
    return "\n".join(s)


def derived(src_lib, src, name, value, fp, desc, retype=None, snap=False):
    """A standard-library symbol body under a new name (same pinout), optionally with pin types changed."""
    lib = Lib(f"{KSYM}/{src_lib}.kicad_sym", src_lib)
    s = lib.flat_local(src)
    s[1] = Q(name)
    for sub in s:
        if isinstance(sub, list) and sub[0] == "property":
            if sub[1] == "Value":
                sub[2] = Q(value)
            elif sub[1] == "Footprint":
                sub[2] = Q(fp)
            elif sub[1] == "ki_description":
                sub[2] = Q(desc)
        if isinstance(sub, list) and sub[0] == "symbol":
            sub[1] = Q(name + sub[1][len(src):])
            for p in sub:
                if isinstance(p, list) and p[0] == "pin" and retype and p[1] in retype:
                    p[1] = retype[p[1]]
                if isinstance(p, list) and p[0] == "pin" and snap:
                    at = next(x for x in p if isinstance(x, list) and x[0] == "at")
                    at[1], at[2] = (f"{round(float(v) / 1.27) * 1.27:g}" for v in at[1:3])
    return dump(s, 1)


def library_text():
    syms = [
        derived("Comparator", "LM2903", "TLV3202", "TLV3202", "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm",
                "TI TLV3202 dual push-pull comparator, 40 ns typ / 55 ns max (SBOS561C); LM2903 pinout (1 OUTA, 2 INA-, 3 INA+, 4 V-, 5 INB+, 6 INB-, 7 OUTB, 8 V+)",
                retype={"open_collector": "output"}),
        derived("74xGxx", "74AUC1G74", "74LVC1G74", "74LVC1G74", "Package_SO:VSSOP-8_2.3x2mm_P0.5mm",
                "TI SN74LVC1G74DCU single D flip-flop with clear and preset (SCES431): 1 CLK, 2 D, 3 Qn, 4 GND, 5 Q, 6 CLRn, 7 PREn, 8 VCC"),
        derived("74xGxx", "74LVC1G332", "74LVC1G332", "74LVC1G332", "Package_TO_SOT_SMD:SOT-23-6",
                "Single 3-input OR gate, SN74LVC1G332 (KiCad 7 library body; input pins moved 2 µm onto the 1.27 mm grid)", snap=True),
        box_symbol("TLV3012", "U", "TLV3012", "Package_TO_SOT_SMD:SOT-23-6",
                   "TI TLV3012 nanopower open-drain comparator with 1.242 V reference, SOT-23-6 (1 OUT, 2 V-, 3 IN+, 4 IN-, 5 REF, 6 V+)",
                   [dict(left=[("3", "IN+", "input"), ("4", "IN-", "input"), ("5", "REF", "output")], right=[("1", "OUT", "open_collector")],
                         top=[("6", "V+", "power_in")], bottom=[("2", "V-", "power_in")])], filt="SOT?23*"),
        box_symbol("UCC27524", "U", "UCC27524", "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm",
                   "TI UCC27524 dual 5 A low-side gate driver, SOIC-8 (1 ENA, 2 INA, 3 GND, 4 INB, 5 OUTB, 6 VDD, 7 OUTA, 8 ENB)",
                   [dict(left=[("1", "ENA", "input"), ("2", "INA", "input"), ("4", "INB", "input"), ("8", "ENB", "input")],
                         right=[("7", "OUTA", "output"), ("5", "OUTB", "output")], top=[("6", "VDD", "power_in")],
                         bottom=[("3", "GND", "power_in")])], filt="SOIC*3.9x4.9mm*"),
        box_symbol("UCC27517", "U", "UCC27517", "Package_TO_SOT_SMD:SOT-23-5",
                   "TI UCC27517 single 4 A low-side gate driver, SOT-23-5 (1 VDD, 2 GND, 3 IN+, 4 IN-, 5 OUT)",
                   [dict(left=[("3", "IN+", "input"), ("4", "IN-", "input")], right=[("5", "OUT", "output")],
                         top=[("1", "VDD", "power_in")], bottom=[("2", "GND", "power_in")])], filt="SOT?23*"),
        box_symbol("LM5069-1", "U", "LM5069-1", "Package_SO:MSOP-10_3x3mm_P0.5mm",
                   "TI LM5069-1 positive hot-swap controller with power limiting, latch-off, MSOP-10 (1 SENSE, 2 VIN, 3 UVLO, 4 OVLO, 5 GND, 6 TIMER, 7 PWR, 8 PGD, 9 OUT, 10 GATE)",
                   [dict(left=[("2", "VIN", "power_in"), ("1", "SENSE", "input"), ("3", "UVLO", "input"), ("4", "OVLO", "input"),
                               ("7", "PWR", "passive"), ("6", "TIMER", "passive")],
                         right=[("10", "GATE", "output"), ("9", "OUT", "input"), ("8", "PGD", "open_collector")],
                         bottom=[("5", "GND", "power_in")])], filt="MSOP*3x3mm*P0.5mm*"),
        box_symbol("AQV252G", "U", "AQV252G", "Package_DIP:DIP-6_W7.62mm",
                   "Panasonic AQV252G PhotoMOS relay, 1 Form A, 60 V 2.5 A, DIP-6 (1 LED+, 2 LED-, 3 NC, 4 out, 5 out common, 6 out)",
                   [dict(left=[("1", "LED+", "passive"), ("2", "LED-", "passive"), ("3", "NC", "no_connect")],
                         right=[("6", "D", "passive"), ("5", "S", "passive"), ("4", "D", "passive")])], filt="DIP*W7.62mm*"),
        box_symbol("ALEV100", "K", "ALEV100", "",
                   "TE Kilovac ALEV100 sealed DC contactor, 100 A, 24 V coil, 1 Form A aux (chassis mount; main A1 = +, polarity marked)",
                   [dict(left=[("A1", "MAIN+", "passive"), ("X1", "COIL+", "passive"), ("3", "AUX", "passive")],
                         right=[("A2", "MAIN", "passive"), ("X2", "COIL-", "passive"), ("4", "AUX", "passive")])]),
        box_symbol("G9SA-321-T", "U", "G9SA-321-T075", "",
                   "Omron G9SA-321-T075 AC/DC24 time-delay safety relay: dual-channel input, 3 NO instantaneous, 2 NO OFF-delay (set 2.0 s), 1 NC aux; DIN rail",
                   [dict(left=[("A1", "A1", "passive"), ("T11", "T11", "passive"), ("T21", "T21", "passive"), ("T31", "T31", "passive"),
                               ("T33", "T33", "passive"), ("13", "13", "passive"), ("23", "23", "passive"), ("33", "33", "passive"),
                               ("41", "41", "passive"), ("47", "47", "passive"), ("57", "57", "passive")],
                         right=[("A2", "A2", "passive"), ("T12", "T12", "passive"), ("T22", "T22", "passive"), ("T32", "T32", "passive"),
                                None, ("14", "14", "passive"), ("24", "24", "passive"), ("34", "34", "passive"),
                                ("42", "42", "passive"), ("48", "48", "passive"), ("58", "58", "passive")])]),
        contact_symbol("MUSHROOM_4NC", "S", "E-stop 4NC",
                       "Two-channel latching mushroom e-stop with four NC contact blocks (a: coil chain, b: logic, c/d: U20 channels); panel mount",
                       [("11", "12"), ("21", "22"), ("31", "32"), ("41", "42")]),
        contact_symbol("PB_2NO", "SB", "ARM 2NO",
                       "ARM / reset pushbutton with two NO contact blocks (a: U10 clock, b: U20 reset); panel mount",
                       [("13", "14"), ("23", "24")], normally="open"),
        contact_symbol("THERMO_NC", "TH", "85C NC",
                       "Bimetal thermal switch, normally closed, opens at 85 C (KSD301 class), bolted to the resistor plate", [("1", "2")]),
        diode_dual_symbol(),
    ]
    return "(kicad_symbol_lib (version 20220914) (generator rnd_kicad_board)\n" + "\n".join(syms) + "\n)\n"
