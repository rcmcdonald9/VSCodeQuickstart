"""Pip v4 hype video: the shared edit (beat grid, shots, cards, impacts). Read by soundtrack.py and compose.py.

120 BPM so a beat is exactly 15 frames at 30 fps; a bar (4 beats) is 2 s. Times are in seconds.
"""
FPS = 30
BPM = 120
BEAT = 60 / BPM
DURATION = 78.0

# (t0, t1, source, src_first_frame, speed, overlay)
#   source: a render shot (rnd/video/render_shots.py), "bench"/"fetch" (twin_footage.py) or "black"
SHOTS = [
    (0.0, 2.0, "black", 0, 1, None),
    (2.0, 6.0, "reveal", 0, 1, None),
    (6.0, 8.0, "glimpse", 0, 1, None),
    (8.0, 10.0, "black", 0, 1, None),
    (10.0, 16.0, "arm", 0, 1, "ARM  //  6 JOINTS + GRIPPER  //  PRINTED BONES"),
    (16.0, 18.0, "arm_end", 0, 1, None),
    (18.0, 22.0, "kx", 0, 1, "KX74 TWIST-LOCK  //  BALLS ON PINS  //  35° TO LOCK"),
    (22.0, 24.0, "kx_end", 0, 1, None),
    (24.0, 28.0, "roll", 0, 1, "SHOULDER ROLL + YAW STAGE  //  BELT DRIVEN"),
    (28.0, 30.0, "roll_end", 0, 1, None),
    (30.0, 34.0, "hand", 0, 1, "HAND  //  WRIST ROLL  //  FIN-RAY FINGERS"),
    (34.0, 36.0, "hand_end", 0, 1, None),
    (36.0, 38.0, "glimpse", 60, 1, None),
    (38.0, 42.0, "bench", 0, 1.08, "DIGITAL TWIN  //  BENCH ARM  //  SCRIPTED PICK-AND-PLACE"),
    (42.0, 46.0, "fetch", 0, 2, "DIGITAL TWIN  //  WHOLE ROBOT FETCH  //  SIM TIME x6"),
    (46.0, 48.0, "black", 0, 1, None),
    (48.0, 56.0, "lift", 0, 1, "MAST LIFT  //  TORSO + BOTH ARMS RIDE THE CARRIAGE"),
    (56.0, 60.0, "glimpse", 0, 1, None),
    (60.0, 62.0, "black", 0, 1, None),
    (62.0, 72.0, "hero", 0, 1, None),
    (72.0, 78.0, "logo", 0, 1, None),
]

# Kinetic cards: (t_in, t_out, lines, style).  Each line: (text, size_px, colour_key).  style: "slam" | "count" | "quiet"
# Every card that is not "quiet" lands on an impact in the soundtrack.
CARDS = [
    (0.35, 1.95, [("EVERY KITCHEN", 64, "w")], "quiet"),
    (1.0, 1.95, [("HAS BEEN WAITING", 64, "o")], "quiet_low"),
    (8.0, 10.0, [("PIP", 330, "w"), ("v4", 150, "o")], "title"),
    (16.0, 18.0, [("ONE ARM.", 150, "w")], "slam"),
    (16.5, 18.0, [("SIX JOINTS.", 150, "w")], "slam2"),
    (17.0, 18.0, [("ZERO CHILL.", 190, "o")], "slam3"),
    (22.0, 24.0, [("TWIST.", 170, "w")], "slam"),
    (22.5, 24.0, [("CLICK.", 170, "w")], "slam2"),
    (23.0, 24.0, [("LOCKED.", 210, "o")], "slam3"),
    (28.0, 30.0, [("±150°", 330, "o"), ("OF YAW. BECAUSE WHY NOT.", 64, "w")], "slam"),
    (34.0, 36.0, [("1 KG", 330, "o"), ("PAYLOAD TARGET. YOUR MUG NEVER STOOD A CHANCE.", 52, "w")], "slam"),
    (36.0, 38.0, [("21", 380, "o"), ("ACTUATORS. ONE ROBOT. ZERO EXCUSES.", 60, "w")], "count"),
    (39.0, 42.0, [("887", 200, "o"), ("SYNTHETIC DEMOS PER HOUR", 50, "w")], "side"),
    (43.0, 46.0, [("IT PRACTISES", 96, "w"), ("WHILE YOU SLEEP.", 96, "o")], "side"),
    (46.0, 48.0, [("≥1.5×", 330, "o"), ("TORQUE MARGIN. ACROSS THE WHOLE WORKSPACE.", 54, "w")], "slam"),
    (50.0, 52.0, [("IT RISES.", 140, "w")], "side"),
    (52.0, 56.0, [("3.4 H", 190, "o"), ("RUNTIME PER CHARGE (EST.)", 48, "w")], "side"),
    (56.0, 57.0, [("43", 330, "o"), ("CHECKS PASSING", 64, "w")], "slam"),
    (57.0, 58.0, [("100%", 300, "o"), ("3D-PRINTED", 64, "w")], "slam"),
    (58.0, 59.0, [("ONE", 300, "o"), ("DESKTOP PRINTER", 64, "w")], "slam"),
    (59.0, 60.0, [("≈$1.5K", 300, "o"), ("BENCH-ARM KIT (ESTIMATE)", 64, "w")], "slam"),
    (60.0, 61.0, [("IT'S NOT BUILT YET.", 96, "w")], "quiet"),
    (61.0, 62.0, [("THE HYPE IS.", 120, "o")], "quiet"),
    (66.0, 72.0, [("PIP", 300, "w"), ("v4", 140, "o")], "hero_title"),
]

# Impacts (big hits): every slam card, the drops, the KX click and the logo.
IMPACTS = sorted({c[0] for c in CARDS if not c[3].startswith("quiet") and c[3] != "side"} | {8.0, 62.0, 72.0, 48.0, 38.0})
SMALL_HITS = [0.35, 1.0, 21.6, 50.0, 52.0, 60.0, 61.0, 39.0, 43.0]
RISERS = [(6.0, 8.0), (60.0, 62.0), (70.0, 72.0), (14.0, 16.0), (20.0, 22.0), (26.0, 28.0), (32.0, 34.0), (44.0, 46.0), (54.0, 56.0)]
KICK_ON = [(8.0, 15.5), (16.0, 21.5), (22.0, 27.5), (28.0, 33.5), (34.0, 45.5), (46.0, 60.0), (62.0, 72.0)]
DROPS = [8.0, 62.0]          # braams + sub drops
CLICK = 21.6                 # KX collar reaches the lock (kx shot frame ~108)
