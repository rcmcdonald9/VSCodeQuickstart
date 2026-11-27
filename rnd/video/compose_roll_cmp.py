"""Side-by-side: current belt roll vs D24 inline GIM8108-36, with the tradeoffs."""
import sys
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

S = Path(sys.argv[1]); OUT = Path(sys.argv[2])
F = "/usr/share/fonts/opentype/inter/Inter-{}.otf"
f = lambda w, s: ImageFont.truetype(F.format(w), s)
W, COL, PAD = 1840, 900, 13
INK, MUTE, GOOD, BAD, BG, LINE = (24, 26, 30), (105, 110, 120), (22, 128, 74), (190, 52, 40), (247, 248, 250), (222, 225, 230)

full = [Image.open(S / f"full_{t}.png").convert("RGB") for t in ("belt", "inline")]
close = [Image.open(S / f"close_{t}.png").convert("RGB") for t in ("belt", "inline")]
ROWS = [  # (label, belt, inline, which is better: 0 belt, 1 inline, None tie)
    ("Shoulder bulge, fore-aft × height (CAD)", "163 × 219 mm", "117 × 100 mm", 1),
    ("Bulge sticking out past the arm joint", "153 mm", "120 mm", 1),
    ("Side grasps across the body (target ≥ 54 %)", "52 %  fails", "62 %  passes", 1),
    ("Shoulder roll speed", "27 rpm", "48 rpm", 1),
    ("Roll torque margin carrying the 20 lb basket", "34×", "52×", 1),
    ("Parts at the roll joint", "80T pulley, belt, tensioner, D13 stage", "motor bolts on, no belt", 1),
    ("Feel when someone pushes the arm by hand", "35.5 : 1 total gearing", "36 : 1, about the same", None),
    ("Roll stage + motor mass, per arm", "1.11 kg", "1.28 kg  (+0.17)", 0),
    ("Motor cost, per arm", "GIM6010-8 + belt parts", "+ ≈ $94 (current listings)", 0),
    ("Known vs. estimated", "every part sourced and checked", "motor size ESTIMATED Ø100 × 60 (no drawing yet)", 0),
    ("Unverified for inline", "—", "motor output bearing must take ≈ 280 N side load", 0),
]
TH = 48 * len(ROWS) + 150
H = 120 + 1150 + 900 + 2 * PAD + TH
im = Image.new("RGB", (W, H), BG); d = ImageDraw.Draw(im)
d.text((PAD + 8, 26), "Shoulder roll: current belt drive vs. inline GIM8108-36", font=f("Bold", 40), fill=INK)
d.text((PAD + 8, 78), "Same robot, same pose and camera; only the shoulder roll drive differs. Simulated from the CAD (rnd/parts/roll.py, ROLL_DRIVE switch).", font=f("Regular", 21), fill=MUTE)
y0 = 120
for i, (a, b) in enumerate(zip(full, close)):
    x = PAD + i * (COL + 14)
    im.paste(a.resize((COL, int(a.height * COL / a.width))), (x, y0))
    im.paste(b.resize((COL, int(b.height * COL / b.width))), (x, y0 + 1150 + PAD))
    tag = ("CURRENT: belt roll (GIM6010-8 + 18T:80T belt)", "OPTION: inline GIM8108-36, bolted on the roll axis")[i]
    d.rounded_rectangle((x + 16, y0 + 16, x + 16 + 14 * len(tag) + 10, y0 + 62), 10, fill=(255, 255, 255))
    d.text((x + 28, y0 + 24), tag, font=f("SemiBold", 22), fill=(INK if i == 0 else GOOD))
    d.rounded_rectangle((x + 16, y0 + 1150 + PAD + 16, x + 300, y0 + 1150 + PAD + 60), 10, fill=(255, 255, 255))
    d.text((x + 28, y0 + 1150 + PAD + 24), "Right shoulder, close-up", font=f("Medium", 21), fill=MUTE)
y = y0 + 1150 + 900 + 2 * PAD + 24
cx = [PAD + 8, 760, 1290]
d.text((cx[0], y), "Tradeoff", font=f("Bold", 24), fill=INK); d.text((cx[1], y), "Current (belt)", font=f("Bold", 24), fill=INK); d.text((cx[2], y), "Inline GIM8108-36", font=f("Bold", 24), fill=INK)
y += 42
for lab, b, n, win in ROWS:
    d.line((PAD, y - 8, W - PAD, y - 8), fill=LINE, width=2)
    d.text((cx[0], y), lab, font=f("Regular", 21), fill=INK)
    for k, (txt, xx) in enumerate(((b, cx[1]), (n, cx[2]))):
        col = INK if win is None else (GOOD if win == k else BAD)
        mark = "" if win is None else ("▲ " if win == k else "▼ ")
        d.text((xx, y), mark + txt, font=f("SemiBold" if win == k else "Regular", 21), fill=col)
    y += 48
d.text((cx[0], y + 10), "▲ better   ▼ worse.  Inline wins on size, reach, speed and part count; it costs ≈ 0.17 kg and ≈ $94 per arm and needs the seller's drawing + bearing rating before buying.",
       font=f("Medium", 20), fill=MUTE)
im.save(OUT, optimize=True)
print(OUT, im.size)
