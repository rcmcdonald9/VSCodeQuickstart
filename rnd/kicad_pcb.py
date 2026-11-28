"""Safety board PCB (stretch goal): footprints from the generated netlist, outline, power-stage placement, power pours,
DRC. Needs KiCad's python module, so it runs under the system interpreter:

    /usr/bin/python3 -m rnd.kicad_pcb          # (python -m rnd.kicad_board --pcb calls it)

Reads hardware/safety_board/safety_board.net (kicad-cli export of the schematic) for footprints, nets and symbol
links (so pcbnew's "Update PCB from Schematic" finds every footprint already linked), places the power stage by hand
(heatsink row of TO-220 / TO-247 parts on the top edge, the clamp loop C2 - stud - MOSFET - diode kept tight) and packs
the rest, pours the high-current nets on both layers (2 oz copper), fills, saves and runs KiCad's DRC
(pcbnew.WriteDRCReport; kicad-cli 7 has no `pcb drc`). Signal routing is NOT done: DRC lists those as unconnected.
"""
from __future__ import annotations

import json
import re
import sys
from pathlib import Path

import pcbnew

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from rnd.kicad_sexp import find, find_all, parse  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "hardware" / "safety_board"
PCB = OUT / "safety_board.kicad_pcb"
KFP = Path("/usr/share/kicad/footprints")
OX, OY = 28.0, 10.0             # board origin in KiCad coordinates (mm)
BW, BH = 240.0, 135.0           # board outline (mm)
CU = 0.070                      # copper thickness, both layers: 2 oz (70 µm)

MM = pcbnew.FromMM


def V(x, y):
    return pcbnew.VECTOR2I(MM(OX + x), MM(OY + y))


# hand placement of the power stage: ref -> (x, y, rotation°) in board coordinates (mm, y down)
FIXED = {
    # heatsink row along the top edge (tabs to the edge; one Al bar clamps them, insulated pads under the drains)
    "Q6": (14, 9, 0), "Q9": (32, 9, 0), "Q10": (50, 9, 0), "D7": (68, 9, 0),
    "Q1": (90, 9, 0), "D4": (108, 9, 0), "Q2": (128, 9, 0), "D5": (146, 9, 0), "Q3": (166, 9, 0), "D6": (184, 9, 0),
    "Q4": (204, 9, 0),
    # studs for the 10 AWG lugs, right under their devices
    "J2": (8, 30, 0), "J4": (24, 30, 0), "J22": (50, 30, 0), "J23": (66, 30, 0),
    "J7": (99, 30, 0), "J8": (137, 30, 0), "J9": (175, 30, 0), "J10": (156, 46, 0), "J5": (210, 30, 0), "J3": (228, 30, 0),
    # bulk and film capacitors closing the clamp loop
    "C2": (118, 46, 0), "C4": (44, 46, 0), "C1": (212, 62, 0),
    # arm branch, base branch, pre-charge
    "U19": (196, 84, 0), "F3": (206, 100, 0), "J11": (228, 84, 0), "J12": (228, 116, 0),
    "R79": (34, 21, 0), "F4": (20, 62, 0), "J20": (8, 76, 0), "J21": (24, 76, 0),
    "K2": (80, 60, 0), "J6": (100, 66, 0),
    "H1": (4, 131, 0), "H2": (236, 131, 0), "H3": (236, 4, 0), "H4": (4, 4, 0),
}
POUR = {  # net -> list of (layer, x0, y0, x1, y1, priority): the high-current copper (board coordinates, mm)
    "F1_OUT": [("F.Cu", 1, 18, 17, 40, 5), ("B.Cu", 1, 18, 17, 40, 5)],
    "VIN": [("F.Cu", 17, 18, 40, 40, 5), ("B.Cu", 17, 18, 40, 40, 5)],
    "VBUS_B": [("F.Cu", 40, 18, 76, 40, 4), ("B.Cu", 40, 18, 76, 40, 4)],
    "DRN_B": [("F.Cu", 45, 24, 56, 36, 6)],
    "DRN1": [("F.Cu", 93, 22, 106, 37, 6)], "DRN2": [("F.Cu", 131, 22, 144, 37, 6)], "DRN3": [("F.Cu", 169, 22, 182, 37, 6)],
    "VBUS": [("F.Cu", 84, 17, 238, 56, 3), ("B.Cu", 84, 17, 238, 56, 3)],
    "PGND": [("B.Cu", 1, 1, 239, 58, 0), ("F.Cu", 84, 1, 232, 17, 1)],
    "GND": [("B.Cu", 1, 60, 239, 134, 0)],      # logic ground plane under the logic; joins PGND only at NT2 (C1-)
}


def board_nets():
    root = parse((OUT / "safety_board.net").read_text())
    comps = {}
    for c in find_all(find(root, "components"), "comp"):
        ref = str(find(c, "ref")[1])
        sp = find(c, "sheetpath")
        tst = find(c, "tstamps")
        comps[ref] = dict(fp=str(find(c, "footprint")[1]), value=str(find(c, "value")[1]),
                          path=str(find(sp, "tstamps")[1]) + str(tst[1]))
    pads = {}
    for n in find_all(find(root, "nets"), "net"):
        name = str(find(n, "name")[1])
        for nd in find_all(n, "node"):
            pads[(str(find(nd, "ref")[1]), str(find(nd, "pin")[1]))] = name
    return comps, pads


def load_fp(fpid):
    lib, name = fpid.split(":")
    fp = pcbnew.FootprintLoad(str(KFP / f"{lib}.pretty"), name)
    if fp is None:
        raise SystemExit(f"footprint {fpid} not found")
    return fp


def bbox_mm(fp):
    b = fp.GetCourtyard(pcbnew.F_CrtYd).BBox() if fp.GetCourtyard(pcbnew.F_CrtYd).OutlineCount() else fp.GetBoundingBox(False, False)
    return (pcbnew.ToMM(b.GetX()), pcbnew.ToMM(b.GetY()), pcbnew.ToMM(b.GetRight()), pcbnew.ToMM(b.GetBottom()))


def rect(board, layer, x0, y0, x1, y1, w=0.1):
    for (a, b), (c, d) in (((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))):
        s = pcbnew.PCB_SHAPE(board)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(V(a, b))
        s.SetEnd(V(c, d))
        s.SetLayer(layer)
        s.SetWidth(MM(w))
        board.Add(s)


def ipc2221_width(i, dt=20.0, t_mm=CU, external=True):
    """IPC-2221 trace width (mm) for current i (A) at a dt (K) rise: I = k·dT^0.44·A^0.725, A in mil²"""
    k = 0.048 if external else 0.024
    area = (i / (k * dt ** 0.44)) ** (1 / 0.725)
    return area / (t_mm / 0.0254) * 0.0254


def main():
    model = json.loads((OUT / "out" / "model.json").read_text())
    mparts = {p["ref"]: p for p in model["parts"]}
    comps, pads = board_nets()
    board = pcbnew.BOARD()
    board.SetCopperLayerCount(2)
    ds = board.GetDesignSettings()
    ds.SetBoardThickness(MM(1.6))
    nets = {}
    for name in sorted(set(pads.values())):
        ni = pcbnew.NETINFO_ITEM(board, name)
        board.Add(ni)
        nets[name] = ni
    rect(board, pcbnew.Edge_Cuts, 0, 0, BW, BH)
    occupied = []
    order = list(FIXED) + [r for r in sorted(comps, key=lambda r: (["power", "clamp", "logic", "base", "conn"].index(mparts[r]["sheet"]),
                                                                    mparts[r]["group"], r)) if r not in FIXED]
    fps = {}
    for ref in order:
        if ref not in comps:
            continue
        c = comps[ref]
        fp = load_fp(c["fp"])
        fp.SetReference(ref)
        fp.SetValue(c["value"])
        fp.SetFPID(pcbnew.LIB_ID(*c["fp"].split(":")))
        fp.SetPath(pcbnew.KIID_PATH(c["path"]))
        for pad in fp.Pads():
            n = pads.get((ref, pad.GetNumber()))
            if n:
                pad.SetNet(nets[n])
        board.Add(fp)
        fps[ref] = fp
    # place: fixed first, then shelf-pack the rest into free space below the power band
    for ref, (x, y, rot) in FIXED.items():
        if ref in fps:
            fps[ref].SetOrientationDegrees(rot)
            fps[ref].SetPosition(V(x, y))
            occupied.append(bbox_mm(fps[ref]))
    # keep the packed (logic) parts off the power pours
    for x0, y0, x1, y1 in ((1, 15, 78, 42), (84, 15, 239, 58)):
        occupied.append((OX + x0, OY + y0, OX + x1, OY + y1))
    gap = 1.0

    def free(b):
        if b[0] < OX + 1 or b[1] < OY + 1 or b[2] > OX + BW - 1 or b[3] > OY + BH - 1:
            return False
        return all(b[2] + gap <= o[0] or b[0] >= o[2] + gap or b[3] + gap <= o[1] or b[1] >= o[3] + gap for o in occupied)
    for ref in order:
        if ref in FIXED or ref not in fps:
            continue
        fp = fps[ref]
        fp.SetPosition(V(0, 0))
        b0 = bbox_mm(fp)
        w, h = b0[2] - b0[0], b0[3] - b0[1]
        ok = False
        y = 40.0
        while y + h < BH - 1 and not ok:
            x = 2.0
            while x + w < BW - 1:
                fp.SetPosition(V(x - (b0[0] - OX), y - (b0[1] - OY)))
                b = bbox_mm(fp)
                if free(b):
                    occupied.append(b)
                    ok = True
                    break
                x += 1.0
            y += 1.0
        if not ok:
            raise SystemExit(f"no room on the board for {ref}")
    # power pours
    for net, zl in POUR.items():
        for layer, x0, y0, x1, y1, prio in zl:
            z = pcbnew.ZONE(board)
            z.SetLayer(pcbnew.F_Cu if layer == "F.Cu" else pcbnew.B_Cu)
            z.SetNet(nets[net])
            z.SetAssignedPriority(prio)
            z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
            z.SetLocalClearance(MM(0.5))
            z.SetMinThickness(MM(0.3))
            ol = z.Outline()
            ol.NewOutline()
            for (a, b) in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
                ol.Append(MM(OX + a), MM(OY + b))
            board.Add(z)
    board.Save(str(PCB))
    # stackup: 2 oz outer copper (text edit of the saved setup block; KiCad reads it back)
    t = PCB.read_text()
    stack = ('(stackup\n      (layer "F.SilkS" (type "Top Silk Screen"))\n      (layer "F.Paste" (type "Top Solder Paste"))\n'
             '      (layer "F.Mask" (type "Top Solder Mask") (thickness 0.01))\n'
             f'      (layer "F.Cu" (type "copper") (thickness {CU}))\n'
             '      (layer "dielectric 1" (type "core") (thickness 1.46) (material "FR4") (epsilon_r 4.5) (loss_tangent 0.02))\n'
             f'      (layer "B.Cu" (type "copper") (thickness {CU}))\n'
             '      (layer "B.Mask" (type "Bottom Solder Mask") (thickness 0.01))\n      (layer "B.Paste" (type "Bottom Solder Paste"))\n'
             '      (layer "B.SilkS" (type "Bottom Silk Screen"))\n      (copper_finish "ENIG")\n      (dielectric_constraints no)\n    )\n    ')
    if "(stackup" not in t:
        t = t.replace("(setup\n    ", "(setup\n    " + stack, 1)
    PCB.write_text(t)
    # reload with the project (net classes), fill, DRC
    board = pcbnew.LoadBoard(str(PCB))
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(str(PCB))
    rpt = OUT / "safety_board_drc.rpt"
    pcbnew.WriteDRCReport(board, str(rpt), pcbnew.EDA_UNITS_MILLIMETRES, True)
    t = rpt.read_text()
    t = re.sub(r"\*\* Created on .*", "** Created by rnd/kicad_pcb.py (pcbnew.WriteDRCReport, KiCad 7.0.11) **", t)
    t = t.replace(str(OUT) + "/", "")
    rpt.write_text(t)
    kinds = {}
    for m in re.finditer(r"^\[(\w+)\]", t, re.M):
        kinds[m.group(1)] = kinds.get(m.group(1), 0) + 1
    summ = re.findall(r"\*\* Found (\d+) (DRC violations|unconnected pads|Footprint errors)", t)
    widths = {n: ipc2221_width(i) for n, i in (("arm branch 20 A", 20.0), ("clamp leg 9.4 A rms", 9.4), ("supply 52 A (trip)", 52.0),
                                               ("supply 37.3 A (robot sustained)", 37.3), ("base 2.2 A sustained / 20 A fuse", 20.0))}
    out = dict(board_mm=[BW, BH], copper_um=CU * 1000, footprints=len(fps), drc=summ, kinds=kinds,
               ipc2221_width_mm_20K_one_layer=widths)
    (OUT / "out" / "pcb_summary.json").write_text(json.dumps(out, indent=1) + "\n")
    import subprocess
    for name, layers in (("front", "F.Cu,F.SilkS,Edge.Cuts"), ("back", "B.Cu,B.SilkS,Edge.Cuts")):
        subprocess.run(["kicad-cli", "pcb", "export", "pdf", "--layers", layers, "-o", str(OUT / f"safety_board_pcb_{name}.pdf"), str(PCB)],
                       capture_output=True)
    print(f"PCB: {BW:.0f} x {BH:.0f} mm, 2 layers, {CU * 1000:.0f} µm copper, {len(fps)} footprints; DRC: {summ}; {kinds}")


if __name__ == "__main__":
    main()
