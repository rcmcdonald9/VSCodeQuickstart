"""KiCad's own ERC on the safety board, headless: KiCad 7.0.11's kicad-cli has no `sch erc` (it arrived in KiCad 8), so
this drives eeschema's ERC dialog in a virtual X server and saves its report.

    apt-get install -y xvfb xdotool imagemagick      # once
    python -m rnd.kicad_erc_gui                       # → hardware/safety_board/safety_board_erc_kicad.rpt

Fragile by nature (it clicks fixed screen positions of the KiCad 7.0.11 GTK layout on a 1600 × 1000 screen); if the
report does not appear, the screenshots in hardware/safety_board/out/erc_gui_*.png show where it stopped. KiCad 8+
replaces all of this with `kicad-cli sch erc`.
"""
from __future__ import annotations

import os
import re
import subprocess
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "hardware" / "safety_board"
RPT = OUT / "safety_board_erc_kicad.rpt"


def x(*args, env):
    return subprocess.run(["xdotool", *args], env=env, capture_output=True, text=True).stdout


def shot(name, env):
    subprocess.run(["import", "-window", "root", str(OUT / "out" / f"erc_gui_{name}.png")], env=env)


def windows(env):
    return x("search", "--name", ".", "getwindowname", "%@", env=env)


def main():
    env = dict(os.environ, DISPLAY=":97")
    xv = subprocess.Popen(["Xvfb", ":97", "-screen", "0", "1600x1000x24"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)
    ee = subprocess.Popen(["eeschema", str(OUT / "safety_board.kicad_sch")], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        for _ in range(60):
            time.sleep(1)
            w = windows(env)
            if "Configure Global Symbol Library Table" in w:      # first run on this machine
                x("mousemove", "1180", "665", "click", "1", env=env)
            if "Schematic Editor" in w:
                break
        time.sleep(4)
        shot("1_open", env)
        x("mousemove", "206", "12", "click", "1", env=env)          # Inspect
        time.sleep(1)
        x("mousemove", "282", "37", "click", "1", env=env)          # Electrical Rules Checker
        time.sleep(3)
        x("mousemove", "923", "540", "click", "1", env=env)         # Run ERC
        time.sleep(15)
        shot("2_result", env)
        if RPT.exists():
            RPT.unlink()
        x("mousemove", "923", "491", "click", "1", env=env)         # Save...
        time.sleep(3)
        x("key", "ctrl+a", env=env)
        x("type", "--delay", "15", str(RPT), env=env)
        x("key", "Return", env=env)
        time.sleep(3)
    finally:
        ee.kill()
        xv.kill()
    if not RPT.exists():
        raise SystemExit("no ERC report written; see hardware/safety_board/out/erc_gui_*.png")
    t = RPT.read_text()
    t = re.sub(r"ERC report \([^)]*\)", "ERC report (eeschema 7.0.11, run by rnd/kicad_erc_gui.py)", t)
    RPT.write_text(t)
    m = re.search(r"ERC messages: (\d+)\s+Errors (\d+)\s+Warnings (\d+)", t)
    print(f"KiCad ERC (eeschema GUI): {m.group(1)} messages, {m.group(2)} errors, {m.group(3)} warnings → {RPT}")


if __name__ == "__main__":
    main()
